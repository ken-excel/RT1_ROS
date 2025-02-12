//RT1 Force con (cmd_tor) with handle
#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Twist.h>
#include "ros_start/Rt1Sensor.h"
#include <pthread.h>
#include <std_msgs/String.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "version.h"

#define MAX_WRENCH_CNT (16)
static geometry_msgs::Wrench last_wrench_buf[MAX_WRENCH_CNT];
static unsigned int last_wrench_cnt_w;
static unsigned int last_wrench_cnt_r;
static int task_end;

int handle_stat; //Added by Ken

typedef struct{
	ros::Subscriber sub_tor_;
	ros::Subscriber sub_vel_;
	ros::Publisher pub_;
#define PORT_BUF_LEN (32)
	char port_[PORT_BUF_LEN];
}TASK_INFO;
static TASK_INFO argsStock;

static double get_double(char *buf, int *pos)
{
	char *ptr;
	double ret;
	int val0, val1, val2;
	int point;
	int sign;
	int i;
	int tmp_pos;

	if ((buf == NULL) ||
		(pos == NULL))
	{
		return 0.0;
	}

	val0 = 0;
	val1 = 0;
	point = -1;
	sign = 1;
	tmp_pos = *pos;

	if (*ptr == ',')
	{
		ptr++;
		tmp_pos++;
	}

	for(ptr=buf;(*ptr!='\0')&&(*ptr!=',');ptr++)
	{
		if (*ptr == '-')
		{
			sign = -sign;
		}
		else if (*ptr == '.')
		{
			point = 0;
		}
		else if ((*ptr >= '0') && (*ptr <= '9'))
		{
			if (point < 0)
			{
				val0 = (val0 * 10) + (int)(*ptr - '0');
			}
			else
			{
				val1 = (val1 * 10) + (int)(*ptr - '0');
				point++;
			}
		}

		tmp_pos++;
	}

	val2 = 1;
	for(i=0;i<point;i++)
	{
		val2 = val2 * 10;
	}

	ret = (double)val0 + ((double)val1 / (double)val2);
	if (sign < 0)
	{
		ret = -ret;
	}

	*pos = tmp_pos;

	return ret;
}


static int get_int(char *buf, int *pos)
{
	char *ptr;
	int ret;
	int val0;
	int sign;
	int tmp_pos;

	if ((buf == NULL) ||
		(pos == NULL))
	{
		return 0.0;
	}

	val0 = 0;
	sign = 1;
	tmp_pos = *pos;
	ptr = &(buf[tmp_pos]);

	if (*ptr == ',')
	{
		ptr++;
		tmp_pos++;
	}

	for(;(*ptr!='\0')&&(*ptr!=',');ptr++)
	{
		if (*ptr == '-')
		{
			sign = -sign;
		}
		else if ((*ptr >= '0') && (*ptr <= '9'))
		{
			val0 = (val0 * 10) + (int)(*ptr - '0');
		}
		tmp_pos++;
	}

	ret = val0;
	if (sign < 0)
	{
		ret = -ret;
	}

	*pos = tmp_pos;

	return ret;
}

static int handle_check(char *buf, int *pos) //Added by Ken
{
	char *ptr;
	int tmp_pos;
	int count;

	if ((buf == NULL) ||
		(pos == NULL))
	{
		return 0.0;
	}

	count = 0;
	tmp_pos = *pos;
	ptr = &(buf[tmp_pos]);

	if (*ptr == ',')
	{
		ptr++;
		tmp_pos++;
	}

	for(;(*ptr!='\0')&&(*ptr!=',');ptr++)
	{
		//fprintf(stderr, "%c\n", *ptr);
		if (*ptr == 'C')
		{
			count++;
		}
		tmp_pos++;
	}

	*pos = tmp_pos;

	return count;
}

static ssize_t write3(int fd, const void *buf, size_t count)
{
	ssize_t ret;

	//printf("write %s", (char *)buf);
	ret = 0;
	ret += write(fd, buf, count);
	ret += write(fd, buf, count);
	ret += write(fd, buf, count);

	return ret;
}

static void *subTask(void *stock)
{
	int ret;
	pthread_t id;
	TASK_INFO *receive_stock;
#define WAIT_SLEEP_NS (10*1000)	/* 10ms */
#define WAIT_100MSEC_CNT_MAX (10)
#define WAIT_1SEC_CNT_MAX (100)
	int wait_1sec_cnt;
	int wait_100msec_cnt;
#define UART_BUF_LEN (256)
	char uart_send_buf[UART_BUF_LEN];
	int uart_send_len;
	char uart_receive_buf[UART_BUF_LEN];
	int uart_receive_len;
	char uart_tmp_buf[UART_BUF_LEN];
	int uart_tmp_len;
	ros_start::Rt1Sensor msg;
	double force, speed, rotate;
	double force_left, force_right;
	double speed_left, speed_right;
	int fspeed, ftorqu, fradiu;
	double latest_speed_left, latest_speed_right;
	double latest_force_left, latest_force_right;

	int fd;
	int pos;
	int i;
	
	receive_stock = (TASK_INFO *)stock;

	uart_send_len = 0;
	uart_receive_len = 0;

	msg.velocity.linear.x = 0.0;
	msg.velocity.linear.y = 0.0;
	msg.velocity.linear.z = 0.0;
	msg.velocity.angular.x = 0.0;
	msg.velocity.angular.y = 0.0;
	msg.velocity.angular.z = 0.0;
	msg.handle.force.x = 0.0;
	msg.handle.force.y = 0.0;
	msg.handle.force.z = 0.0;
	msg.handle.torque.x = 0.0;
	msg.handle.torque.y = 0.0;
	msg.handle.torque.z = 0.0;
	msg.accel.linear.x = 0.0;
	msg.accel.linear.y = 0.0;
	msg.accel.linear.z = 0.0;
	msg.accel.angular.x = 0.0;
	msg.accel.angular.y = 0.0;
	msg.accel.angular.z = 0.0;

	latest_speed_left = 0.0;
	latest_speed_right = 0.0;
	latest_force_left = 0.0;
	latest_force_right = 0.0;

	/* UART open */
	fd = open(receive_stock->port_, O_RDWR | O_NONBLOCK);
if (fd < 0) fprintf(stderr, "open %s error\n", receive_stock->port_);

	/* UART???M */
	strcpy(uart_send_buf, "setmode dbg rtw12345\r\n");
	write3(fd, uart_send_buf, strlen(uart_send_buf));
	strcpy(uart_send_buf, "fdrive1\r\n");
	write3(fd, uart_send_buf, strlen(uart_send_buf));
	strcpy(uart_send_buf, "fturn1\r\n");
	write3(fd, uart_send_buf, strlen(uart_send_buf));
	strcpy(uart_send_buf, "fspeed2048\r\n");
	write3(fd, uart_send_buf, strlen(uart_send_buf));
	strcpy(uart_send_buf, "ftorqu2048\r\n");
	write3(fd, uart_send_buf, strlen(uart_send_buf));
	strcpy(uart_send_buf, "fradiu2048\r\n");
	write3(fd, uart_send_buf, strlen(uart_send_buf));
	strcpy(uart_send_buf, "mtlr2\r\n");
	write3(fd, uart_send_buf, strlen(uart_send_buf));
	strcpy(uart_send_buf, "mtrr2\r\n");
	write3(fd, uart_send_buf, strlen(uart_send_buf));
	strcpy(uart_send_buf, "sar2\r\n");
	write3(fd, uart_send_buf, strlen(uart_send_buf));
	strcpy(uart_send_buf, "syr2\r\n");
	write3(fd, uart_send_buf, strlen(uart_send_buf));
	strcpy(uart_send_buf, "sfr2\r\n");
	write3(fd, uart_send_buf, strlen(uart_send_buf));

	wait_100msec_cnt = WAIT_100MSEC_CNT_MAX;
	wait_1sec_cnt = 0;

	for(;task_end == 0;)
	{
		if (last_wrench_cnt_w != last_wrench_cnt_r)
		{
			if (handle_stat == 2){
				force = last_wrench_buf[last_wrench_cnt_r].force.x;		/* N */
				rotate = last_wrench_buf[last_wrench_cnt_r].torque.z;	/* N.m */
				
				//printf("wrench %g %g\n", force, rotate);

				last_wrench_cnt_r++;
				if (last_wrench_cnt_r >= MAX_WRENCH_CNT)
				{
					last_wrench_cnt_r = 0;
				}

				if((rotate == 0.0)&&(force == 0.0))
				{
					ftorqu = 0;
					fradiu = 0;
				}
				else
				{
	#define REAR_TREAD_RT1	(0.42)		/*!< ?쓮?ւ̃g???b?h[m] */
					force_left = force / 2.0 - rotate / REAR_TREAD_RT1;		/* N */
					force_right = force / 2.0 + rotate / REAR_TREAD_RT1;	/* N */

	#define FORCE_COEFF_RT1 (69.66)		/*!< ????ϐ??ւ̕ϊ??W?? */
					force_left *= FORCE_COEFF_RT1;	/* mA */
					force_right *= FORCE_COEFF_RT1;	/* mA */

					if (rotate > 0.0)
					{
						ftorqu = (int)force_right;
						fradiu = (int)(1000.0 * force_left / force_right) - 1000;
					}
					else
					{
						ftorqu = (int)force_left;
						fradiu = 1000 - (int)(1000.0 * force_right / force_left);
					}
				}

				if (ftorqu > 2000)
				{
					fprintf(stderr, "ftorqu %d over 2000. Correct 2000\n", ftorqu);
					ftorqu = 2000;
				}
				if (ftorqu < -2000)
				{
					fprintf(stderr, "ftorqu %d under -2000. Correct -2000\n", ftorqu);
					ftorqu = -2000;
				}
				if (fradiu > 2000)
				{
					fprintf(stderr, "fradiu %d over 2000. Correct 2000\n", fradiu);
					fradiu = 2000;
				}
				if (fradiu < -2000)
				{
					fprintf(stderr, "fradiu %d under -2000. Correct -2000\n", fradiu);
					fradiu = -2000;
				}

				if (ftorqu == 0)
				{
					/* ftorqu = 0??fspeed?Ő??䂳?????[?h??w?????邱?ƂɂȂ??ŉ?????*/
					ftorqu = 1;
				}

				ftorqu += 2048;
				fradiu += 2048;

				/* UART???M */
				sprintf(uart_send_buf, "ftorqu%c%c%c%c\r\n",
						((ftorqu / 1000) % 10) + '0',
						((ftorqu / 100) % 10) + '0',
						((ftorqu / 10) % 10) + '0',
						((ftorqu / 1) % 10) + '0');
				write3(fd, uart_send_buf, strlen(uart_send_buf));
				sprintf(uart_send_buf, "fradiu%c%c%c%c\r\n",
						((fradiu / 1000) % 10) + '0',
						((fradiu / 100) % 10) + '0',
						((fradiu / 10) % 10) + '0',
						((fradiu / 1) % 10) + '0');
				write3(fd, uart_send_buf, strlen(uart_send_buf));
				wait_1sec_cnt = WAIT_1SEC_CNT_MAX;	/* Set counter to stop RT.1 after 1 sec. */
			}

			else if (handle_stat == 0){
				speed = last_wrench_buf[last_wrench_cnt_r].force.x;		/* m/s */
				rotate = last_wrench_buf[last_wrench_cnt_r].torque.z;	/* rad/s */
				
				last_wrench_cnt_r++;
				if (last_wrench_cnt_r >= MAX_WRENCH_CNT)
				{
					last_wrench_cnt_r = 0;
				}

	#define REAR_TREAD_RT1	(0.42)		/*!< ‹ì“®—Ö‚ÌƒgƒŒƒbƒh[m] */
				speed_left = speed - rotate * REAR_TREAD_RT1 / 2.0;		/* m/s */
				speed_right = speed + rotate * REAR_TREAD_RT1 / 2.0;		/* m/s */

	#define SPEED_COEFF_RT1 (311.111)	/*!< “à•”•Ï”‚Ö‚Ì•ÏŠ·ŒW” */
				speed_left *= SPEED_COEFF_RT1;
				speed_right *= SPEED_COEFF_RT1;

				if (rotate > 0.0)
				{
					fspeed = (int)speed_right;
					fradiu = (int)(1000.0 * speed_left / speed_right) - 1000;
				}
				else
				{
					fspeed = (int)speed_left;
					fradiu = 1000 - (int)(1000.0 * speed_right / speed_left);
				}

				if (fspeed >= 2000)
				{
					fprintf(stderr, "fspeed %d over 2000. Correct 2000\n", fspeed);
					fspeed = 2000;
				}
				if (fspeed < -2000)
				{
					fprintf(stderr, "fspeed %d under -2000. Correct -2000\n", fspeed);
					fspeed = -2000;
				}
				if (fradiu >= 2000)
				{
					fprintf(stderr, "fradiu %d over 2000. Correct 2000\n", fradiu);
					fradiu = 2000;
				}
				if (fradiu < -2000)
				{
					fprintf(stderr, "fradiu %d under -2000. Correct -2000\n", fradiu);
					fradiu = -2000;
				}

				fspeed += 2048;
				fradiu += 2048;

				/* UART‘—M */
				sprintf(uart_send_buf, "fspeed%c%c%c%c\r\n",
						((fspeed / 1000) % 10) + '0',
						((fspeed / 100) % 10) + '0',
						((fspeed / 10) % 10) + '0',
						((fspeed / 1) % 10) + '0');
				write3(fd, uart_send_buf, strlen(uart_send_buf));
				sprintf(uart_send_buf, "fradiu%c%c%c%c\r\n",
						((fradiu / 1000) % 10) + '0',
						((fradiu / 100) % 10) + '0',
						((fradiu / 10) % 10) + '0',
						((fradiu / 1) % 10) + '0');
				write3(fd, uart_send_buf, strlen(uart_send_buf));

				wait_1sec_cnt = WAIT_1SEC_CNT_MAX;	/* Set counter to stop RT.1 after 1 sec. */
			}

			else{
				strcpy(uart_send_buf, "fspeed2048\r\n");
				write3(fd, uart_send_buf, strlen(uart_send_buf));
				strcpy(uart_send_buf, "ftorqu2048\r\n");
				write3(fd, uart_send_buf, strlen(uart_send_buf));
				strcpy(uart_send_buf, "fradiu2048\r\n");
				write3(fd, uart_send_buf, strlen(uart_send_buf));
			}
		}

		uart_tmp_len = read(fd, uart_tmp_buf, UART_BUF_LEN);
		if (uart_tmp_len >= 1)	/* UART?? */
		{
			for(i=0;i<uart_tmp_len;i++)
			{
				uart_receive_buf[uart_receive_len++] = uart_tmp_buf[i];

				if (uart_receive_len >= UART_BUF_LEN)
				{
					uart_receive_len = UART_BUF_LEN-1;
				}

				if (uart_tmp_buf[i] == '\n')
				{
					uart_receive_buf[uart_receive_len] = '\0';
					//printf("read %d %s\n", uart_receive_len, uart_receive_buf);

					if ((uart_receive_buf[0] == 'm') &&
						(uart_receive_buf[1] == 't') &&
						(uart_receive_buf[2] == 'l'))
					{
						/* Store values into RtSensor structure */
						pos = 3;
						latest_speed_left = (double)get_int(uart_receive_buf, &pos) / 3600.0;	/* m/h -> m/s */
					}
					else if ((uart_receive_buf[0] == 'm') &&
							 (uart_receive_buf[1] == 't') &&
							 (uart_receive_buf[2] == 'r'))
					{
						/* Store values into RtSensor structure */
						pos = 3;
						latest_speed_right = (double)get_int(uart_receive_buf, &pos) / 3600.0;	/* m/h -> m/s */
					}
					else if ((uart_receive_buf[0] == 's') &&
							 (uart_receive_buf[1] == 'a'))
					{
						/* Store values into RtSensor structure */
						/*
						  x = y;
						  y = -x;
						  z = z;
						 */
						pos = 2;
						get_int(uart_receive_buf, &pos);
						get_int(uart_receive_buf, &pos);
						msg.accel.linear.y = (double)get_int(uart_receive_buf, &pos) / -1000.0;	/* mm/s^2 -> m/s^2 */
						get_int(uart_receive_buf, &pos);
						get_int(uart_receive_buf, &pos);
						msg.accel.linear.x = (double)get_int(uart_receive_buf, &pos) / 1000.0;	/* mm/s^2 -> m/s^2 */
						get_int(uart_receive_buf, &pos);
						msg.accel.linear.z = (double)get_int(uart_receive_buf, &pos) / 1000.0;	/* mm/s^2 -> m/s^2 */
					}
					else if ((uart_receive_buf[0] == 's') &&
							 (uart_receive_buf[1] == 'y'))
					{
						/* Store values into RtSensor structure */
						/*
						  x = y;
						  y = -x;
						  z = z;
						 */
						pos = 2;
						msg.accel.angular.y = (double)get_int(uart_receive_buf, &pos) / -10000.0;	/* 0.0001rad/s -> rad/s */
						msg.accel.angular.x = (double)get_int(uart_receive_buf, &pos) / 10000.0;	/* 0.0001rad/s -> rad/s */
						msg.accel.angular.z = (double)get_int(uart_receive_buf, &pos) / 10000.0;	/* 0.0001rad/s -> rad/s */
					}
					else if ((uart_receive_buf[0] == 's') &&
							 (uart_receive_buf[1] == 'f'))
					{
						/* Store values into RtSensor structure */
						pos = 2;
						latest_force_left = (double)get_int(uart_receive_buf, &pos) / 1000.0;	/* mN -> N */
						latest_force_right = (double)get_int(uart_receive_buf, &pos) / 1000.0;	/* mN -> N */
					}
					//printf("%g %g %g %g %g %g %g %g %g %g\n", msg.velocity.linear.x, msg.velocity.angular.z, msg.handle.force.x, msg.handle.torque.z, msg.accel.linear.x, msg.accel.linear.y, msg.accel.linear.z, msg.accel.angular.x, msg.accel.angular.y, msg.accel.angular.z);

					else if ((uart_receive_buf[0] == 's') &&
							 (uart_receive_buf[1] == 'h'))
					{
						/* Store values into RtSensor structure */
						pos = 2;
						handle_stat = get_int(uart_receive_buf, &pos);
						handle_stat = get_int(uart_receive_buf, &pos);
						handle_stat = get_int(uart_receive_buf, &pos);
						handle_stat = get_int(uart_receive_buf, &pos);
						handle_stat = handle_check(uart_receive_buf, &pos); //get handle
						//fprintf(stderr, "Handle:%d\n", handle_stat);
					}

					uart_receive_len = 0;
				}
				else if (uart_tmp_buf[i] < 0x20)
				{
					continue;
				}
			}
		}

		if (wait_1sec_cnt >= 1)
		{
			wait_1sec_cnt--;

			if (wait_1sec_cnt == 0)
			{
				/* UART???M */
				strcpy(uart_send_buf, "fspeed2048\r\n");
				write3(fd, uart_send_buf, strlen(uart_send_buf));
				strcpy(uart_send_buf, "ftorqu2048\r\n");
				write3(fd, uart_send_buf, strlen(uart_send_buf));
			}
		}

		if (wait_100msec_cnt >= 1)
		{
			wait_100msec_cnt--;
			if (wait_100msec_cnt == 0)
			{
				/* publish RtSensor structure */

				msg.velocity.linear.x = (latest_speed_left + latest_speed_right) / 2.0;
				msg.velocity.angular.z = (latest_speed_right - latest_speed_left) / REAR_TREAD_RT1;

#define SENSOR_GAP_RT1	(0.16)		/*!< ???̓Z???T?Ԃ̋???[m] */
				msg.handle.force.x = (latest_force_left + latest_force_right) / 2.0;
				msg.handle.torque.z = (latest_force_right - latest_force_left) / SENSOR_GAP_RT1;

				/* Some variable may not be updated, but don't care */
				receive_stock->pub_.publish(msg);

				wait_100msec_cnt = WAIT_100MSEC_CNT_MAX;
			}
		}

		usleep(WAIT_SLEEP_NS);	/* 10ms */
	}

	strcpy(uart_send_buf, "fdrive0\r\n");
	write3(fd, uart_send_buf, strlen(uart_send_buf));
	strcpy(uart_send_buf, "mtlr1\r\n");
	write3(fd, uart_send_buf, strlen(uart_send_buf));
	strcpy(uart_send_buf, "mtrr1\r\n");
	write3(fd, uart_send_buf, strlen(uart_send_buf));
	strcpy(uart_send_buf, "sar1\r\n");
	write3(fd, uart_send_buf, strlen(uart_send_buf));
	strcpy(uart_send_buf, "syr1\r\n");
	write3(fd, uart_send_buf, strlen(uart_send_buf));
	strcpy(uart_send_buf, "sfr1\r\n");
	write3(fd, uart_send_buf, strlen(uart_send_buf));

	/* UART close */
	close(fd);

	id = pthread_self();
	pthread_detach( (pthread_t)id );
	pthread_exit(&ret);

	return NULL;
}

class Rosrt_Rt1
{
public:
	Rosrt_Rt1() {
		ros::NodeHandle node;

		ros::NodeHandle nh("~");
		std::string port_param;
		// Default value version
		nh.param<std::string>("port", port_param, "/dev/rt1");

		argsStock.sub_tor_ = node.subscribe("cmd_tor", 1, &Rosrt_Rt1::wrenchCallback, this);
		argsStock.sub_vel_ = node.subscribe("cmd_vel", 1, &Rosrt_Rt1::twistCallback, this);
		argsStock.pub_ = node.advertise<ros_start::Rt1Sensor>("rosrt_rt1", 1);
		strncpy(argsStock.port_, port_param.c_str(), PORT_BUF_LEN);

		task_end = 0;

		if (pthread_create(&id, NULL, subTask, (void*)&argsStock) != 0)
		{
			fprintf(stderr, "TaskCreate returns NULL. try one more time.\n");
		}

		if (id == 0)
		{
			fprintf(stderr, "TaskCreate returns NULL. try one more time.\n");
		}
	}

	~Rosrt_Rt1() {

		task_end = -1;

		pthread_cancel ((pthread_t)id);         /* Send cancel request   */
		pthread_join((pthread_t)id, NULL);
	}

	void twistCallback(const geometry_msgs::Twist &twist_msg) {
		if(handle_stat == 0){ //Allow twist only when the person is not operating
			geometry_msgs::Wrench wrench_msg; //Fake wrench msg containing twist msg
			wrench_msg.force.x = twist_msg.linear.x;
			wrench_msg.torque.z = twist_msg.angular.z;
			last_wrench_buf[last_wrench_cnt_w] = wrench_msg;
			last_wrench_cnt_w++;
			if (last_wrench_cnt_w >= MAX_WRENCH_CNT)
			{
				last_wrench_cnt_w = 0;
			}
		}
	}

	void wrenchCallback(const geometry_msgs::Wrench &wrench_msg) {
		if(handle_stat == 2){
	//		printf("wrenchCallback\n");
			last_wrench_buf[last_wrench_cnt_w] = wrench_msg;
			last_wrench_cnt_w++;
			if (last_wrench_cnt_w >= MAX_WRENCH_CNT)
			{
				last_wrench_cnt_w = 0;
			}
		}
	}

private:
	pthread_t id;
};

int main(int argc, char **argv)
{
	printf("rosrt_rt1 : version %s\n", VERSION_NUMBER);

	last_wrench_cnt_w = 0;
	last_wrench_cnt_r = 0;

	ros::init(argc, argv, "rosrt_rt1");
	Rosrt_Rt1 rosrt_rt1;
	ros::spin();
}

