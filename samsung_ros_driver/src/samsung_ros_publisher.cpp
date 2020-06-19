#include <mutex>

#include <stdio.h>
#include <unistd.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <string.h>
#include <pthread.h>
#include <thread>

#include <cyusb.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/CameraInfo.h>

#include <samsung_event_msgs/Event.h>
#include <samsung_event_msgs/EventArray.h>


typedef void * (*THREADFUNCPTR)(void *);

class SamsungDVSPublisher {
public:
    SamsungDVSPublisher();
    ~SamsungDVSPublisher();
    void run();

private:
    struct Node {
        void *data;
        int len;
        struct Node* next;
    };
    struct Node* front = NULL;
    struct Node* rear = NULL;
    pthread_mutex_t queue_mutex = PTHREAD_MUTEX_INITIALIZER;
    int32_t queue_size = 0;
    uint64_t packets_received = 0;

    /// \brief Opens the camera
    bool openCamera();

    void* camera_stream_loop(void*);
    void* processor_loop(void*);

    void Enqueue(void *pkt, int len);
    void *Dequeue(int *len);

    /// \brief Node handler - the access point to communication with ROS
    ros::NodeHandle nh_;

    /// \brief Publisher for camera info
    ros::Publisher pub_info_;

    /// \brief Publisher for CD events
    ros::Publisher pub_events_;

    /// \brief Message for publishing the camera info
    sensor_msgs::CameraInfo cam_info_msg_;

    /// \brief Path to the file with the camera settings (biases)
    std::string biases_file_;

    /// \brief Maximum events rate, in kEv/s
    int max_event_rate_;


    /// \brief Loading configuration
    int htoi(char s[], int *i);
    int I2cValueLen (int slvAddr);
    int writeI2cReg(int slvAddr, int addr, int val);
    int parseString(char *s, int *slvAddr, int *adr, int *val);
    bool loadScript(const char *s);


    std::string camera_name;
    bool running = false;
    bool debug = false;

    ros::Time start_timestamp_;
    cyusb_handle *h1 = NULL;
    int buflen = -1;

    static const int I2C_VALUE_LEN = 2;
    static const int I2C_SLAVE_ADDR = 0x60;    //96;    // 0x60

    static const int I2C_SLAVE_ADDR_DVSL = 0x20;
    static const int I2C_SLAVE_ADDR_DVSR = 0x30;
    static const int I2C_SLAVE_ADDR_D2FX = 0x40;
    static const int I2C_SLAVE_ADDR_M2PR = 0x1A;
    static const int I2C_SLAVE_ADDR_M2PL = 0x1C;
    static const int I2C_VALUE_LEN_DVSL = 1;
    static const int I2C_VALUE_LEN_DVSR = 1;
    static const int I2C_VALUE_LEN_D2FX = 1;
    static const int I2C_VALUE_LEN_M2PR = 2;
    static const int I2C_VALUE_LEN_M2PL = 2;
    const int MESSAGE_DT = 20000;    // 20msec
    const int timeout = 1000;
};


SamsungDVSPublisher::SamsungDVSPublisher():
    nh_("~"),
    biases_file_(""),
    max_event_rate_(30000),
    camera_name("camera")
{
    // Load Parameters
    nh_.getParam("camera_name", camera_name);
    nh_.getParam("bias_file", biases_file_);

    const std::string topic_cam_info = "/samsung/" + camera_name + "/camera_info";
    const std::string topic_event_buffer = "/samsung/" + camera_name + "/events";

    pub_info_ = nh_.advertise<sensor_msgs::CameraInfo>(topic_cam_info, 1);
    pub_events_ = nh_.advertise<samsung_event_msgs::EventArray>(topic_event_buffer, -1);

    while (!openCamera()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        ROS_INFO("Trying to open Samsung camera...");
    }

    ROS_INFO("Width: %i, Height: %i", 640, 480);

    if (biases_file_ != "")
        loadScript(biases_file_.c_str());

    // Publish camera info message
    cam_info_msg_.width = 640;
    cam_info_msg_.height = 480;
    cam_info_msg_.header.frame_id = "SamsungCamera_optical_frame";
}


int SamsungDVSPublisher::htoi(char s[], int *i)
{
	int hexdigit;
	int n = 0;
	char c;

	while (true) {
		c = s[*i];

		if(c >='0' && c <='9')
			hexdigit = c - '0';
		else if(c >='a' && c <='f')
			hexdigit = c -'a' + 10;
		else if(c >='A' && c <='F')
			hexdigit = c -'A' + 10;
		else
			return n;

		n = 16 * n + hexdigit;

		*i = *i + 1;
	}
}


int SamsungDVSPublisher::I2cValueLen (int slvAddr) {
	switch(slvAddr){
		case I2C_SLAVE_ADDR_D2FX : return(I2C_VALUE_LEN_D2FX);
		case I2C_SLAVE_ADDR_DVSL : return(I2C_VALUE_LEN_DVSL);
		case I2C_SLAVE_ADDR_DVSR : return(I2C_VALUE_LEN_DVSR);
		case I2C_SLAVE_ADDR_M2PL : return(I2C_VALUE_LEN_M2PL);
		case I2C_SLAVE_ADDR_M2PR : return(I2C_VALUE_LEN_M2PR);
		default : return(I2C_VALUE_LEN_D2FX);
	}
}


int SamsungDVSPublisher::writeI2cReg(int slvAddr, int addr, int val)
{
	int r;
	unsigned char buf[I2C_VALUE_LEN];

	if (debug) ROS_INFO("[I] writeI2cReg(%X,%X,%X)\n", slvAddr, addr, val);

	int	I2c_Value_Len = I2cValueLen (slvAddr);

	if (I2c_Value_Len == 1) {
		buf[0] = val & 0xff;
	} else {
		buf[1] = val & 0xff;
		buf[0] = (val >> 8) & 0xff;
	}

	r = cyusb_control_transfer(h1, 0x40, 0xBA, slvAddr, addr, buf, I2c_Value_Len, timeout);

	if (r != I2c_Value_Len ) {
		ROS_ERROR("Error writing I2C register\n");
			return -1;
	}

	return 0;
}


int SamsungDVSPublisher::parseString(char *s, int *slvAddr, int *adr, int *val) {
	int i = 0;
	int *dst;
	int mode = 10, ret;

	dst = slvAddr;
	while (true) {
		switch (tolower(s[i])) {
			case 0   : return mode;
			case '\n': return mode;
			case '/' : return mode;

			case ' ' : break;
			case '\t': break;
			case ':' : break;
			case '=' : break;

			case 'w':
				if (tolower(s[i+1]) == 'a' && tolower(s[i+2]) == 'i' && tolower(s[i+3]) == 't') {
					i = i + 3;
					mode = 20;
					dst = val;
					break;
				} else {
					return -1;
				}

			case '0':
			case '1':
			case '2':
			case '3':
			case '4':
			case '5':
			case '6':
			case '7':
			case '8':
			case '9':
			case 'a':
			case 'b':
			case 'c':
			case 'd':
			case 'e':
			case 'f':
				*dst = htoi(s, &i);
				if (debug) ROS_INFO("[I] Mode(%d): Value(%X) : (%X, %X, %X)\n", mode, *dst, *slvAddr, *adr, *val);
				switch (mode) {
					case 10 :
						mode = 11;
						dst = adr;
						break;
					case 11 :
						mode = 12;
						dst = val;
						break;
					case 12 :
						mode = 1;
						break;
					case 20 :
						mode = 2;
						break;
					default :
						break;
				}
				break;

			default: return -1;
		}
		i++;
	}
}


bool SamsungDVSPublisher::loadScript(const char *s)
{
	int slvAddr, adr, val;

	if (debug) ROS_INFO("[I] loadScript(%s)\n", s);

	const int buflen = 1000;
	char buf[buflen];
	FILE * file;

	file = fopen(s , "r");
	if (!file) {
		ROS_ERROR("Error opening file %s\n", s);
		return false;
	}

	while (fgets(buf, buflen, file)!=NULL) {

		if (debug) ROS_INFO("%s",buf);

		switch (parseString(buf, &slvAddr, &adr, &val)) {
			case 1:
				if (debug) ROS_INFO("[I] I2C (%X,%X,%X)\n", slvAddr, adr, val);
				writeI2cReg(slvAddr, adr, val);
				break;
			case 2:
				if (debug) ROS_INFO("[I] WAIT (%d)\n", val);
				usleep(val*1000);
				break;
			default: continue;
		}
	}

	fclose(file);
    return true;
}


SamsungDVSPublisher::~SamsungDVSPublisher() {
    cyusb_close();
    nh_.shutdown();
}


bool SamsungDVSPublisher::openCamera() {
    auto r = cyusb_open();
    if ( r < 0 ) {
        ROS_WARN("Error opening library\n");
        return false;
    } else if ( r == 0 ) {
        ROS_WARN("No device found\n");
        return false;
    }
    if ( r > 1 ) {
        ROS_WARN("More than 1 devices of interest found. Disconnect unwanted devices\n");
        return false;
    }

    h1 = cyusb_gethandle(0);
    if ( cyusb_getvendor(h1) != 0x04b4 ) {
        ROS_WARN("Cypress chipset not detected\n");
        cyusb_close();
        return false;
    }
    r = cyusb_kernel_driver_active(h1, 0);
    if ( r != 0 ) {
        ROS_WARN("kernel driver active!\n");
        cyusb_close();
        return false;
    }
    r = cyusb_claim_interface(h1, 0);
    if ( r != 0 ) {
        ROS_WARN("Error in claiming interface\n");
        cyusb_close();
        return false;
    }

    buflen = cyusb_get_max_packet_size(h1, 0x81);
    ROS_INFO("Buffer size: %d Bytes\n", buflen);
    if (buflen <= 1024) ROS_WARN("Please check the USB connection (USB-3.0 or higher is recommended)\n");

    return true;
}


void SamsungDVSPublisher::run() {
    running = true;

    pthread_t tidStream, tidProcess;
    auto r = pthread_create(&tidStream, NULL, (THREADFUNCPTR) &SamsungDVSPublisher::camera_stream_loop, this);
    r = pthread_create(&tidProcess, NULL, (THREADFUNCPTR) &SamsungDVSPublisher::processor_loop, this);

    ros::Rate loop_rate(5);
    while(ros::ok()) {
        if (pub_info_.getNumSubscribers() > 0) {
            cam_info_msg_.header.stamp = ros::Time::now();
            pub_info_.publish(cam_info_msg_);
        }
        loop_rate.sleep();
    }

    running = false;
    pthread_join(tidStream, NULL);
    pthread_join(tidProcess, NULL);
}


void* SamsungDVSPublisher::camera_stream_loop(void*) {
    int r;
    void *buf;
    int transferred = 0;
    int nPkt = 0;

    while (running) {
        buf = malloc(buflen);
        r = cyusb_bulk_transfer(h1, 0x81, (unsigned char*)buf, buflen, &transferred, timeout * 1000);
        if ( r == 0 ) {
            if (debug) ROS_INFO("Received packet %d, len=%d", nPkt, buflen);
            Enqueue(buf, buflen);
            nPkt++;

            if (packets_received % 1000 == 0) {
                ROS_INFO("Samsung Camera packets received: %lu", packets_received);
            }

            continue;
        } else {
            cyusb_error(r);
            ROS_ERROR("Samsung Camera: %s Packet Receive Failure!", camera_name.c_str());
            cyusb_close();
            running = false;
            return NULL;
        }

    }
}


void *SamsungDVSPublisher::processor_loop(void*) {
    unsigned char *buf;
    int pktlen;
    int nPkt = 0;

    // Parsing
    int                    i, n;
    int                    posX, posY, posY0, grpAddr, header;
    bool polarity = false;
    bool initialized = false;
    unsigned int        longTs = 0;
    unsigned int        shortTs = 0;
    int64_t eTsOffset = 0;
    int64_t timeStamp = 0;
    int64_t lastMessageTs = 0;
    int64_t lastEtimeStamp = 0;
    uint64_t nFramesReceived = 0;

    if (debug) ROS_INFO("Processor thread started\n");

    start_timestamp_ = ros::Time::now();

    samsung_event_msgs::EventArray event_buffer_msg;
    event_buffer_msg.height = 480;
    event_buffer_msg.width  = 640;

    event_buffer_msg.events.reserve(2 * max_event_rate_);
    while (running) {
        buf = (unsigned char *)Dequeue(&pktlen);
        if (buf == NULL)
            continue;

        // process
        if (debug) ROS_INFO("Dequeued packet %d, len=%d\n", nPkt++, pktlen);

        if (pktlen < 4) continue;
        if (pktlen % 4) pktlen -= pktlen % 4;

        for (i = 0; i < pktlen; i += 4) {
            header = buf[i] & 0x7C;
            //sensorID = buff[i] & 0x03;

            if (buf[i] & 0x80) {    // Group Events Packet

                grpAddr = (buf[i+1] & 0xFC) >> 2;

                if (buf[i + 3]) {
                    posY0 = grpAddr << 3;
                    polarity = (buf[i+1] & 0x01)? true : false;
                    for (n=0; n<8; n++) {
                        if ((buf[i+3] >> n) & 0x01) {
                            posY = posY0 + n;

                            samsung_event_msgs::Event event;
                            event.x = posX;
                            event.y = posY;
                            event.polarity = polarity;
                            event.ts.fromNSec(start_timestamp_.toNSec() + (timeStamp - eTsOffset) * 1000.00);
                            event_buffer_msg.events.push_back(event);
                        }
                    }
                }

                if (buf[i + 2]) {
                    grpAddr += (header >> 2);    // Offset
                    posY0 = grpAddr << 3;
                    polarity = (buf[i+1] & 0x02)? true : false;
                    for (n=0; n<8; n++) {
                        if ((buf[i+2] >> n) & 0x01) {
                            posY = posY0 + n;

                            samsung_event_msgs::Event event;
                            event.x = posX;
                            event.y = posY;
                            event.polarity = polarity;
                            event.ts.fromNSec(start_timestamp_.toNSec() + (timeStamp - eTsOffset) * 1000.00);
                            event_buffer_msg.events.push_back(event);
                        }
                    }
                }

                if (!initialized) {
                    event_buffer_msg.events.clear();
                }

            } else {
                switch (header) {
                    case (0x04) :    // 0000 01** | --ST TTTT | TTTT T-CC | CCCC CCCC    Column Address (10) + SubTimestamp (10)
                        if (buf[i+1] & 0x20) {    // Update timestamp only once when new frame starts
                            shortTs = ((buf[i+1] & 0x1F) << 5) | ((buf[i+2] & 0xF8) >> 3);
                            timeStamp = longTs + shortTs;

                            //if (initialized)
                            //    std::cout << "ts:" << timeStamp << "\n";

                            if (timeStamp < lastEtimeStamp) {
                                eTsOffset = timeStamp;
                                start_timestamp_ = ros::Time::now();
                                if (initialized) {
                                    ROS_ERROR("Samsung Camera: %ld, %ld timestamps are not in order!", lastEtimeStamp, timeStamp);
                                }
                                initialized = true;
                            }
                            if (!initialized && ((ros::Time::now() - start_timestamp_).toSec() > 2)) { // 2 sec
                                //std::cout << nFramesReceived << " " << timeStamp << " " << eTsOffset << "\n";
                                eTsOffset = timeStamp;
                                start_timestamp_ = ros::Time::now();
                                initialized = true;
                            }

                            lastEtimeStamp = timeStamp;
                            nFramesReceived ++;
                        }
                        posX = (((buf[i + 2] & 0x03) << 8) | (buf[i + 3] & 0xFF));        // Original
                        break;

                    case (0x08) :    // 0000 10** | --TT TTTT | TTTT TTTT | TTTT TTTT    Reference Timestamp (22)
                        longTs = (((buf[i + 1] & 0x3F) << 16) | ((buf[i + 2] & 0xFF) << 8) | (buf[i + 3] & 0xFF)) * 1000;
                        break;

                    case (0x40) :    // 0100 00** | --II IIII | IIII IIII | IIII IIII    Packet ID (22)
                        // Packet ID is used to check packet loss (MIPI)
                        //packetID = ((buf[i + 1] & 0x3F) << 26) | ((buf[i + 2] & 0xFF) << 18) | ((buf[i + 3] & 0xFF) << 10);
                        break;

                    case (0x00) :    // 0000 0000 | 0000 0000 | 0000 0000 | 0000 0000    Padding (MIPI)
                        //i = pktlen;    // ignore all the remaining packet data
                        break;

                    default :
                        break;
                }
            }
        }

        if ((event_buffer_msg.events.size() > 0) &&((timeStamp - lastMessageTs > MESSAGE_DT) || (event_buffer_msg.events.size() > max_event_rate_))) {
            lastMessageTs = timeStamp;
            //event_buffer_msg.header.stamp = event_buffer_msg.events[0].ts;
            event_buffer_msg.header.stamp.fromNSec(start_timestamp_.toNSec() + (timeStamp - eTsOffset) * 1000.00);
            pub_events_.publish(event_buffer_msg);
            event_buffer_msg.events.clear();
        }

        free(buf);
    }
}


void SamsungDVSPublisher::Enqueue(void *pkt, int len) {
    pthread_mutex_lock(&queue_mutex);

    struct Node* temp = (struct Node*)malloc(sizeof(struct Node));
    temp->data = pkt;
    temp->len = len;
    temp->next = NULL;

    if(front == NULL && rear == NULL){
        front = rear = temp;
    } else {
        rear->next = temp;
        rear = temp;
    }
    queue_size ++;
    pthread_mutex_unlock(&queue_mutex);

    packets_received ++;
    if (debug) ROS_INFO("Enqueued packet, len=%d\n", len);
}


void *SamsungDVSPublisher::Dequeue(int *len) {
    *len = 0;
    void *data = NULL;
    pthread_mutex_lock(&queue_mutex);

    struct Node* temp = front;
    if(front == NULL) {
        //if (debug) ROS_INFO("Queue is Empty\n");
    } else {
        if(front == rear) {
            data = front->data;
            *len = front->len;
            front = rear = NULL;
        } else {
            data = front->data;
            *len = front->len;
            front = front->next;
        }
        queue_size --;
        free(temp);
    }

    if (queue_size > 0) {
        ROS_WARN("Samsung Camera event queue lag: %d", queue_size);
    }

    pthread_mutex_unlock(&queue_mutex);
    return data;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "samsung_ros_publisher");

    SamsungDVSPublisher s;
    s.run();

    ros::shutdown();

    return 0;
}
