//
// Created by cuiqiang on 2017/1/3.
//
#include "com_ling_jiboserialport_SerialPortUtil.h"

/* Header for class com_netposa_jni_MUtils */

#ifdef __cplusplus
extern "C" {
#endif

#include <string.h>
#include <android/log.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include "termios.h"
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <stdlib.h>

#define FALSE -1
#define TRUE 0

#define MISC ("/dev/block/by-name/misc")
#define BUFF_SIZE (32)
#define SN_OFFSET (-32)

char BUF[BUFF_SIZE];

#define UART_DEVICE  "/dev/ttyS0"
#define SPEED_LEFT 3
#define SPEED_RIGHT 3

#define TAG "ai.ling.jni" // 这个是自定义的LOG的标识
#define ALOGD(...)  __android_log_print(ANDROID_LOG_DEBUG,TAG,__VA_ARGS__) // 定义LOGD类型
#define ALOGI(...)  __android_log_print(ANDROID_LOG_DEBUG,TAG,__VA_ARGS__) // 定义LOGD类型
#define ALOGE(...)  __android_log_print(ANDROID_LOG_DEBUG,TAG,__VA_ARGS__) // 定义LOGD类型

static int uart_fd;
unsigned char rcv_buf[512];

const unsigned char turnLeft[] = { 0xff, 0x00, 0x04, 0x00, 0xff };
const unsigned char turnRight[] = { 0xff, 0x00, 0x03, 0x00, 0xff };
const unsigned char forward[] = { 0xff, 0x00, 0x01, 0x00, 0xff };
const unsigned char goBack[] = { 0xff, 0x00, 0x02, 0x00, 0xff };
const unsigned char stop[] = { 0xff, 0x00, 0x00, 0x00, 0xff };

const unsigned char moveHand[] = { 0xff, 0x13, 0x00, 0x00, 0xff };
const unsigned char moveRedBarrier[] = { 0xff, 0x13, 0x01, 0x00, 0xff };

const unsigned char openLight[] = { 0xff, 0x05, 0x00, 0x00, 0xff };
const unsigned char closeLight[] = { 0xff, 0x05, 0x01, 0x00, 0x00 };

unsigned char speedLeft[] = { 0xff, 0x02, 0x02, 0x00, 0xff };
unsigned char speedRight[] = { 0xff, 0x02, 0x01, 0x00, 0xff };

int uartOpen(char *uartDevice) {
	int fd;
	fd = open(uartDevice, O_RDWR | O_NOCTTY | O_NDELAY);
	if (FALSE == fd) {
		perror("Can't Open Serial Port");
		return FALSE;
	}

	//判断串口的状态是否为阻塞状态
	if (fcntl(fd, F_SETFL, 0) < 0) {
		ALOGI("fcntl failed!\n");
		return FALSE;
	} else {
		ALOGI("fcntl=%d\n", fcntl(fd, F_SETFL, 0));
	}
	//测试是否为终端设备
//		if (0 == isatty(STDIN_FILENO)) {
//			ALOGI("standard input is not a terminal device\n");
//			return FALSE;
//		}
	return fd;
}

void uartClose(int fd) {
	close(fd);
}

int uartSet(int fd, int speed, int flow_ctrl, int databits, int stopbits,
		int parity) {
	unsigned int i;
	int speed_arr[] = { B115200, B38400, B19200, B9600, B4800, B2400, B1200,
			B300 };
	int name_arr[] = { 115200, 38400, 19200, 9600, 4800, 2400, 1200, 300 };
	struct termios options;
	if (tcgetattr(fd, &options) != 0) {
		perror("SetupSerial 1");
		return (FALSE);
	}
	for (i = 0; i < sizeof(speed_arr) / sizeof(int); i++) {
		if (speed == name_arr[i]) {
			cfsetispeed(&options, speed_arr[i]);
			cfsetospeed(&options, speed_arr[i]);
		}
	}

	options.c_cflag |= CLOCAL;
	options.c_cflag |= CREAD;
	switch (flow_ctrl) {
	case 0: //不使用流控制
		options.c_cflag &= ~CRTSCTS;
		break;
	case 1: //使用硬件流控制
		options.c_cflag |= CRTSCTS;
		break;
	case 2: //使用软件流控制
		options.c_cflag |= IXON | IXOFF | IXANY;
		break;
	}
	options.c_cflag &= ~CSIZE; //屏蔽其他标志位
	switch (databits) {
	case 5:
		options.c_cflag |= CS5;
		break;
	case 6:
		options.c_cflag |= CS6;
		break;
	case 7:
		options.c_cflag |= CS7;
		break;
	case 8:
		options.c_cflag |= CS8;
		break;
	default:
		fprintf(stderr, "Unsupported data size\n");
		return (FALSE);
	}
	switch (parity) {
	case 'n':
	case 'N': //无奇偶校验位。
		options.c_cflag &= ~PARENB;
		options.c_iflag &= ~INPCK;
		break;
	case 'o':
	case 'O': //设置为奇校验
		options.c_cflag |= (PARODD | PARENB);
		options.c_iflag |= INPCK;
		break;
	case 'e':
	case 'E': //设置为偶校验
		options.c_cflag |= PARENB;
		options.c_cflag &= ~PARODD;
		options.c_iflag |= INPCK;
		break;
	case 's':
	case 'S': //设置为空格
		options.c_cflag &= ~PARENB;
		options.c_cflag &= ~CSTOPB;
		break;
	default:
		fprintf(stderr, "Unsupported parity\n");
		return (FALSE);
	}
	switch (stopbits) {
	case 1:
		options.c_cflag &= ~CSTOPB;
		break;
	case 2:
		options.c_cflag |= CSTOPB;
		break;
	default:
		fprintf(stderr, "Unsupported stop bits\n");
		return (FALSE);
	}
	options.c_oflag &= ~OPOST;
	options.c_cc[VTIME] = 1;
	options.c_cc[VMIN] = 1;
	tcflush(fd, TCIFLUSH);
	if (tcsetattr(fd, TCSANOW, &options) != 0) {
		perror("com set error!/n");
		return (FALSE);
	}
	return (TRUE);
}

int uartInit(int fd, int speed, int flow_ctrlint, int databits, int stopbits,
		char parity) {
	//设置串口数据帧格式
	if (FALSE == uartSet(fd, speed, flow_ctrlint, databits, stopbits, parity)) {
		return FALSE;
	} else {
		return TRUE;
	}
}

int uartRecv(int fd, unsigned char *rcv_buf, int data_len) {

	int len, fs_sel;
	fd_set fs_read;

	struct timeval time;

	FD_ZERO(&fs_read);
	FD_SET(fd, &fs_read);

	time.tv_sec = 10;
	time.tv_usec = 0;
	fs_sel = select(fd + 1, &fs_read, NULL, NULL, &time);
	if (fs_sel) {
		len = read(fd, rcv_buf, data_len);
		return len;
	} else {
		return FALSE;
	}
}

int uartSend(int fd, const unsigned char *send_buf, int data_len) {

	int ret;
	ret = write(fd, send_buf, data_len);
	if (data_len == ret) {
		return ret;
	} else {
		tcflush(fd, TCOFLUSH);
		return FALSE;
	}

}

int uartSendCommand(const unsigned char *command, int commandSize) {

	int i, ret = TRUE, fd = FALSE;
	fd = uartOpen(UART_DEVICE);
	if (FALSE == fd) {
		ALOGI("Open error\n");
		return FALSE;
	}
	ret = uartInit(fd, 9600, 0, 8, 1, 'N');
	if (FALSE == ret) {
		ALOGI("Set Port Error\n");
		return FALSE;
	}
	ret = uartSend(fd, command, commandSize);
	if (FALSE == ret) {
		ALOGI("Send error\n");
		return FALSE;
	}

	uartClose(fd);
	return ret;
}
/*
 * Class:     com_ling_jiboserialport_SerialPortUtil
 * Method:    getSerialNumber
 * Signature: ()Ljava/lang/String;
 */
JNIEXPORT jstring JNICALL Java_com_ling_jiboserialport_SerialPortUtil_getSerialNumber(
		JNIEnv * env, jclass cls) {

	int fd;
	int size;
	int err;
	if ((fd = open(MISC, O_RDWR)) < 0) {
		return NULL;
	}
	memset(BUF, 0, BUFF_SIZE);
	lseek(fd, SN_OFFSET, SEEK_END);
	err = read(fd, BUF, BUFF_SIZE);
	if (err <= 0) {
		perror("hecy:read failed\n");
		return NULL;
	}

	close(fd);
	size = strnlen(BUF, BUFF_SIZE);
	if (size < 1) {
		ALOGE("%s %d read strlen < 0.\n", __func__, __LINE__);
		return NULL;
	}

	ALOGD("%s %d buf = %s\n", __func__, __LINE__, BUF);

	jclass clsstring = (*env)->FindClass(env, "java/lang/String");
	jstring strencode = (*env)->NewStringUTF(env, "utf-8");
	jmethodID mid = (*env)->GetMethodID(env, clsstring, "<init>",
			"([BLjava/lang/String;)V");
	jbyteArray barr = (*env)->NewByteArray(env, size);
	(*env)->SetByteArrayRegion(env, barr, 0, size, (jbyte *) BUF);
	return (jstring)(*env)->NewObject(env, clsstring, mid, barr, strencode);
}

/*
 * Class:     com_ling_jiboserialport_SerialPortUtil
 * Method:    setSerialNumber
 * Signature: (Ljava/lang/String;)Z
 */
JNIEXPORT jboolean JNICALL Java_com_ling_jiboserialport_SerialPortUtil_setSerialNumber(
		JNIEnv * env, jclass cls, jstring serialNumber) {

	int fd;
	if ((fd = open(MISC, O_RDWR)) < 0) {
		ALOGE("%s %d File Open Error\n", __func__, __LINE__);
		return 0;
	}
	jclass clsstring = (*env)->FindClass(env, "java/lang/String");
	jstring strencode = (*env)->NewStringUTF(env, "utf-8");
	jmethodID mid = (*env)->GetMethodID(env, clsstring, "getBytes",
			"(Ljava/lang/String;)[B");
	jbyteArray barr = (jbyteArray)(*env)->CallObjectMethod(env, serialNumber,
			mid, strencode);
	jsize alen = (*env)->GetArrayLength(env, barr);
	jbyte *ba = (*env)->GetByteArrayElements(env, barr, JNI_FALSE);

	ALOGD("%s %d strlen = %d\n", __func__, __LINE__, alen);

	if (alen > 0 && alen <= 32) {
		memset(BUF, 0, BUFF_SIZE);
		memcpy(BUF, ba, alen);
		BUF[alen] = 0;
	} else {
		return 0;
	}

	(*env)->ReleaseByteArrayElements(env, barr, ba, 0);

	lseek(fd, SN_OFFSET, SEEK_END);
	write(fd, BUF, BUFF_SIZE);
	close(fd);

	return 1;
}

/*
 * Class:     com_ling_jiboserialport_SerialPortUtil
 * Method:    turnLeft
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_com_ling_jiboserialport_SerialPortUtil_turnLeft
(JNIEnv * env, jclass cls) {

	int ret;
	ret = uartSendCommand(turnLeft, sizeof(turnLeft));
	if(ret == FALSE) {
		ALOGI("MJni %s %d Send Command Error\n", __func__, __LINE__);
	}
	return;

}

/*
 * Class:     com_ling_jiboserialport_SerialPortUtil
 * Method:    turnRight
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_com_ling_jiboserialport_SerialPortUtil_turnRight
(JNIEnv * env, jclass cls) {

	int ret;
	ret = uartSendCommand(turnRight, sizeof(turnRight));
	if(ret == FALSE) {
		ALOGI("MJni %s %d Send Command Error\n", __func__, __LINE__);
	}
	return;
}

/*
 * Class:     com_ling_jiboserialport_SerialPortUtil
 * Method:    forward
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_com_ling_jiboserialport_SerialPortUtil_forward
(JNIEnv * env, jclass cls) {

	int ret;
	ret = uartSendCommand(forward, sizeof(forward));
	if(ret == FALSE) {
		ALOGI("MJni %s %d Send Command Error\n", __func__, __LINE__);
	}
	return;
}

/*
 * Class:     com_ling_jiboserialport_SerialPortUtil
 * Method:    goback
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_com_ling_jiboserialport_SerialPortUtil_goback
(JNIEnv * env, jclass cls) {

	int ret;
	ret = uartSendCommand(goBack, sizeof(goBack));
	if(ret == FALSE) {
		ALOGI("MJni %s %d Send Command Error\n", __func__, __LINE__);
	}
	return;
}

/*
 * Class:     com_ling_jiboserialport_SerialPortUtil
 * Method:    stop
 * Signature: (I)V
 */
JNIEXPORT void JNICALL Java_com_ling_jiboserialport_SerialPortUtil_stop
(JNIEnv *env, jclass cls) {

	int ret;
	ret = uartSendCommand(stop, sizeof(stop));
	if(ret == FALSE) {
		ALOGI("MJni %s %d Send Command Error\n", __func__, __LINE__);
	}
	return;
}

/*
 * Class:     com_ling_jiboserialport_SerialPortUtil
 * Method:    moveHand
 * Signature: (I)V
 */
JNIEXPORT void JNICALL Java_com_ling_jiboserialport_SerialPortUtil_moveHand
(JNIEnv *env, jclass cls) {

	int ret;
	ret = uartSendCommand(moveHand, sizeof(moveHand));
	if(ret == FALSE) {
		ALOGI("MJni %s %d Send Command Error\n", __func__, __LINE__);
	}
	return;
}

/*
 * Class:     com_ling_jiboserialport_SerialPortUtil
 * Method:    moveBarrier
 * Signature: (I)V
 */
JNIEXPORT void JNICALL Java_com_ling_jiboserialport_SerialPortUtil_moveBarrier
(JNIEnv *env, jclass cls) {

	int ret;
	ret = uartSendCommand(moveRedBarrier, sizeof(moveRedBarrier));
	if(ret == FALSE) {
		ALOGI("MJni %s %d Send Command Error\n", __func__, __LINE__);
	}
	return;
}

/*
 * Class:     com_ling_jiboserialport_SerialPortUtil
 * Method:    openLight
 * Signature: (I)V
 */
JNIEXPORT void JNICALL Java_com_ling_jiboserialport_SerialPortUtil_openLight
(JNIEnv *env, jclass cls) {

	int ret;
	ret = uartSendCommand(openLight, sizeof(openLight));
	if(ret == FALSE) {
		ALOGI("MJni %s %d Send Command Error\n", __func__, __LINE__);
	}
	return;
}

/*
 * Class:     com_ling_jiboserialport_SerialPortUtil
 * Method:    closeLight
 * Signature: (I)V
 */
JNIEXPORT void JNICALL Java_com_ling_jiboserialport_SerialPortUtil_closeLight
(JNIEnv *env, jclass cls) {

	int ret;
	ret = uartSendCommand(closeLight, sizeof(closeLight));
	if(ret == FALSE) {
		ALOGI("MJni %s %d Send Command Error\n", __func__, __LINE__);
	}
	return;
}


/*
 * Class:     com_ling_jiboserialport_SerialPortUtil
 * Method:    setSpeedLeft
 * Signature: (I)V
 */
JNIEXPORT void JNICALL Java_com_ling_jiboserialport_SerialPortUtil_setSpeedLeft
(JNIEnv * env, jclass cls, jint speed){

	int ret;
	speedLeft[SPEED_LEFT] = speed;
	ret = uartSendCommand(speedLeft, sizeof(speedLeft));
	if(ret == FALSE) {
		ALOGI("MJni %s %d Send Command Error\n", __func__, __LINE__);
	}

	return;
}

/*
 * Class:     com_ling_jiboserialport_SerialPortUtil
 * Method:    setSpeedRight
 * Signature: (I)V
 */
JNIEXPORT void JNICALL Java_com_ling_jiboserialport_SerialPortUtil_setSpeedRight
(JNIEnv * env, jclass cls, jint speed) {

	int ret;
	speedRight[SPEED_RIGHT] = speed;
	ret = uartSendCommand(speedRight, sizeof(speedRight));
	if(ret == FALSE) {
		ALOGI("MJni %s %d Send Command Error\n", __func__, __LINE__);
	}

	return;
}

#ifdef __cplusplus
}
#endif

