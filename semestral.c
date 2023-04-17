#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <termios.h>
#include <unistd.h>
#include <pthread.h>
#include "prg_serial_nonblock.h"
#include "messages.h"
#include "event_queue.h"
#include "xwin_sdl.h" // visualistaion library

#define SERIAL_READ_TIMOUT_MS 500
#define DEFAUL_CHUNKSIZE 30
#define MAX_CHUNKSIZE 250
#define DEFAULT_ITER_NUMBER 60
#define DEFAULT_WIDTH 320
#define DEFAULT_HEIGHT 240
#define DEFAULT_A_REAL -1.6 // start point real
#define DEFAULT_A_IMAG -1.1 // start point imag
#define DEFAULT_B_REAL 1.1
#define DEFAULT_B_IMAG 1.1
#define DEFAULT_C_REAL -0.4
#define DEFAULT_C_IMAG 0.6



typedef enum //parameters to be changed
{
	C_REAL, C_IMAG, WIDTH, HEIGHT, N, NO_PARAM
} param;

typedef struct // pixel (image made of pixel vector)
{
	uint8_t r, g, b;
} pix;

typedef struct // chunk (part of image, used by nucleo)
{
	uint8_t H, W, cid;
	pix *data;
} chunk;

typedef struct // image
{
	pix* data; // pixel array (image as vector)
	int H, W;
	uint8_t number_of_chunks, chunksinrow, chunksize;
	bool refresh_requested; // after size change
} image;

typedef struct // computation data
{
	uint8_t n,  current_chunk;
	double aR, aI; // start
	double bR, bI; // end
	double cR, cI; // parameter C
	double stepR, stepI; // size of step
    bool computing, computation_set;
	param parameter_to_change;
}computation;

typedef struct // shared data
{
   bool quit;
   int fd;
} data_t;

char* change_parameters_name[] = {"C_REAL", "C_IMAGINARY", "WIDTH", "HEIGHT", "ITERATIONS"}; // names for user only

pthread_mutex_t mtx;
pthread_cond_t cond;

void call_termios(int reset);
void* input_thread(void*);
void* serial_rx_thread(void*);
bool get_message(unsigned char msg_type, data_t *data, uint8_t *buffer, event *ev);
bool send_message(data_t *data, message *msg); 
uint8_t get_chunk_size(int width, int height);
image* init_image(int WIDTH, int HEIGHT,  int chunksize);
image* dark_image(image* d);
image* compute_on_pc(image* img, computation parameters);
image* pc_animation(image* img, computation parameters);
void print_settings();
void print_help();
void print_plus_minus();
double get_red(double t);
double get_green(double t);
double get_blue(double t);


// -- boss function -- //
int main(int argc, char *argv[])
{
	data_t data = { .quit = false, .fd = -1};
	const char *serial = argc > 1 ? argv[1] : "/dev/ttyACM0";
	data.fd = serial_open(serial);

	if (data.fd == -1) // check serial open
	{
		fprintf(stderr, "ERROR: Cannot open serial port %s\n", serial);
		exit(100);
	}
	
	pthread_mutex_init(&mtx, NULL); // initialize mutex with default attributes
	pthread_cond_init(&cond, NULL); // initialize mutex with default attributes
	call_termios(0);

	enum { INPUT, SERIAL_RX, NUM_THREADS };
	const char *threads_names[] = { "Input", "Serial In" };
	void* (*thr_functions[])(void*) = {input_thread, serial_rx_thread};
	pthread_t threads[NUM_THREADS];
	for (int i = 0; i < NUM_THREADS; ++i)  // threads creation
	{
		int r = pthread_create(&threads[i], NULL, thr_functions[i], &data);
		fprintf(stderr, "INFO: Create thread '%s' %s\n", threads_names[i], ( r == 0 ? "OK" : "FAIL") );
	}
	
	fprintf(stderr, "\n");
	fprintf(stderr, "INFO: Press 'h' to see help\n");
	fprintf(stderr, "\n");

	computation parameters = {.n = DEFAULT_ITER_NUMBER, .cR = DEFAULT_C_REAL, .cI = DEFAULT_C_IMAG, .aR = DEFAULT_A_REAL, .aI = DEFAULT_A_IMAG, .bR = DEFAULT_B_REAL,
	.bI = DEFAULT_B_IMAG, .computing = false, .computation_set = false, .current_chunk = -1, .parameter_to_change = NO_PARAM};

	message msg;
	image* img = init_image(DEFAULT_WIDTH, DEFAULT_HEIGHT, DEFAUL_CHUNKSIZE);
	bool quit = false;
	int h, w;

	while(!quit) // event handeling
	{

		xwin_poll_events();
		event ev = queue_pop(); // get event from queue

		if(ev.source == EV_KEYBOARD) // keyboard event
		{
			msg.type = MSG_NBR; // default message type
			switch(ev.type)
			{
				case EV_GET_VERSION: // get version from nucleo
					msg.type = MSG_GET_VERSION;
					fprintf(stderr, "INFO: Get version requested\n");
					break;
				case EV_CLEAR_BUFFER: // clear image
					if (img->refresh_requested)
						fprintf(stderr, "WARN: Refresh the image\n\r");
					else if (parameters.computing)
						fprintf(stderr,"WARN: Ongoing computation, buffer not cleared\n\r");
					else
					{
						img = dark_image(img);
						parameters.current_chunk = -1;
						fprintf(stderr,"INFO: Chung id reset, buffer cleared\n\r");
					}
					break;
				case EV_RESET_CHUNK: // set chunk to zero (default is -1)
					if (parameters.computing)
						fprintf(stderr, "WARN: Ongoing computation, change not saved\n\r");
					else
					{
						parameters.current_chunk = -1;
						fprintf(stderr, "INFO: Chunk reset request\n\r");
					}
					break;
				case EV_ABORT: // stop computation
					if (!parameters.computing)
						fprintf(stderr, "WARN: No ongoing computation\n\r");	
					else
					{
						msg.type = MSG_ABORT;
						parameters.current_chunk--;
						parameters.computing = false;
						fprintf(stderr, "INFO: PC abort\n\r");
					}
					break;
				case EV_CHANGE_PARAMETER: // set parameter to be changed
					parameters.parameter_to_change = ev.data.param;
					fprintf(stderr, "INFO: Parameter %s has been picked for change\r\n", change_parameters_name[ev.data.param]);
					print_plus_minus();
					break;
				case EV_CHANGE: // change of selected parameter
					switch(parameters.parameter_to_change)
					{
						case NO_PARAM:
							fprintf(stderr, "WARN: No parametr selected\r\n");
							break;
						case C_REAL:
							if(ev.data.param == 1)
								parameters.cR += 0.005;
							else
								parameters.cR -= 0.005;
							fprintf(stderr, "INFO: Updated variable C = %.2f + i %.2f (real part)\n\r",parameters.cR, parameters.cI);
							break;
						case C_IMAG:
							if(ev.data.param == 1)
								parameters.cR += 0.005;
							else
								parameters.cR -= 0.005;
							fprintf(stderr, "INFO:: Updated variable C = %.2f + i %.2f (complex part)\n\r",parameters.cR, parameters.cI);
							break;
						case WIDTH:
							if (!parameters.computing)
							{
								img->refresh_requested = true;
								if(ev.data.param == 1)
									img->W += 20;
								else
									img->W -= 20;
								fprintf(stderr, "INFO: WIDTH changed: %d x %d , refresh the image\n\r",img->W, img->H);
							}
							else
								fprintf(stderr, "WARN: Is not able change image size during computation\r\n");
							break;
						case HEIGHT:
							if (!parameters.computing)
							{
								img->refresh_requested = true;
								if(ev.data.param == 1)
									img->H += 20;
								else
									img->H -= 20;
								fprintf(stderr, "INFO: HEIGHT changed: %d x %d , refresh the image\n\r",img->W, img->H);
							}
							else
								fprintf(stderr, "WARN: Is not able change image size during computation\r\n");
							break;
						case N:
							if(ev.data.param == 1 && parameters.n < 100)
								parameters.n += 1;
							else if(parameters.n > 1)
								parameters.n -= -1;
							fprintf(stderr, "INFO: Updated iteration number = %d \n\r",parameters.n );
							break;	
					}
					parameters.computation_set = false;
					break;
				case EV_SET_COMPUTE:
					if (!parameters.computing)
					{
						msg.type = MSG_SET_COMPUTE;
						parameters.computation_set = true;
						parameters.stepR = (parameters.bR-parameters.aR) / img->H;  // step for each pixel in row
						parameters.stepI = (parameters.bI-parameters.aI) / img->H;  // step for each pixel in column
						// data to message
						msg.data.set_compute.c_re = parameters.cR;
						msg.data.set_compute.c_im = parameters.cI;
						msg.data.set_compute.d_re = parameters.stepR; 			
						msg.data.set_compute.d_im = parameters.stepI;
						msg.data.set_compute.n = parameters.n;
						fprintf(stderr,"INFO: Computation parameters set\n\r");
					}
					else
						fprintf(stderr,"WARN: Ongoing computation, parameters not set\n");
					break;
				case EV_COMPUTE:
					if (img->refresh_requested)
						fprintf(stderr, "WARN: Size of image has been change, refresh the image\n\r");
					else if (parameters.computing)
						fprintf(stderr, "WARN: Computation is already running\n\r");
					else if (!parameters.computation_set)
						fprintf(stderr, "Set parameters first\n\r");
					else
					{
						msg.type = MSG_COMPUTE;
						parameters.computing = true;
						parameters.current_chunk++;
						msg.data.compute.cid = parameters.current_chunk;
						msg.data.compute.n_re = get_chunk_size(img->W, img->H);
						msg.data.compute.n_im = get_chunk_size(img->W, img->H);
						//           starting point         +               how many steps done                            * step
						//           general starting point + (number of chunk % number of chunks on line) * size of chunk * step
						msg.data.compute.re = parameters.aR + (parameters.current_chunk % img->chunksinrow) * img->chunksize * parameters.stepR; // get start coodrs
						msg.data.compute.im = parameters.aI + (parameters.current_chunk / img->chunksinrow) * img->chunksize * parameters.stepI; // get start coodrs
						fprintf(stderr,"INFO: New computation chunk id: %d for part %d x %d\n", msg.data.compute.cid, msg.data.compute.n_re, msg.data.compute.n_im);
					}
					break;
				case EV_COMPUTE_PC:
					if (img->refresh_requested)
						fprintf(stderr, "WARN: Size of image has been change, refresh the image\n\r");
					else if (!parameters.computation_set)
						fprintf(stderr, "WARN: Computation was requested, but parameters has not been set up yet\n\r");
					else
					{
						img = compute_on_pc(img, parameters);
						fprintf(stderr, "INFO: Computation was provided on computer\n\r");
					}
					break;
				case EV_REFRESH_IMAGE:
					h = img->H;
					w = img->W;
					xwin_close();
					free(img); // frees image memory
					img = init_image(w, h, get_chunk_size(w, h));
					xwin_redraw(img->W, img->H, (unsigned char*)img->data);
					parameters.current_chunk = -1; // 0, because ++1 in comunication with nucleo
					fprintf(stderr, "INFO: Image refreshed\r\n");
					break;
				case EV_SHOW_ANIMATION:
					if(img->refresh_requested)
						fprintf(stderr, "WARN: Refresh image\n\r");
					else if (parameters.computing)
						fprintf(stderr, "WARN: Ongoing computation, try later\r\n");
					else
					{
						fprintf(stderr, "INFO: Animation created\r\n");
						img = pc_animation(img, parameters);
					}
					break;
				default:
					break;
			}
			if(msg.type != MSG_NBR)
			{
				if (!send_message(&data, &msg))
					fprintf(stderr, "ERROR: send_message() does not send all bytes of the message!\n");
			}
		}
		

		else if (ev.source == EV_NUCLEO) // nucleo event
		{
			if (ev.type == EV_SERIAL) 
			{
				message *msg = ev.data.msg;
				switch (msg->type) 
				{
					case MSG_STARTUP:
					{
						char str[STARTUP_MSG_LEN + 1];
						for (int i = 0; i < STARTUP_MSG_LEN; ++i)
							str[i] = msg->data.startup.message[i];
						str[STARTUP_MSG_LEN] = '\0';
						parameters.computing = false;
						parameters.computation_set = false;
						parameters.current_chunk = -1;
						fprintf(stderr, "INFO: Nucleo restarted - '%s'\n", str);
						break;
					}
					case MSG_VERSION:
						if (msg->data.version.patch > 0) 
							fprintf(stderr, "INFO: Nucleo firmware ver. %d.%d-p%d\n", msg->data.version.major, msg->data.version.minor, msg->data.version.patch);
						else
							fprintf(stderr, "INFO: Nucleo firmware ver. %d.%d\n", msg->data.version.major, msg->data.version.minor);
						break;
					case MSG_ERROR:
						fprintf(stderr, "WARN: Receive error from Nucleo\r\n");
						break;
					case MSG_OK:
						fprintf(stderr, "INFO: Receive ok from Nucleo\r\n");
						break;
					case MSG_ABORT:
						fprintf(stderr, "INFO: Abort from Nucleo\r\n");
						if (!parameters.computing)
							fprintf(stderr, "WARN: Abort from NUCLEO is requested but it is not computing\n\r");	
						else
						{
							parameters.computing = false;
							parameters.current_chunk--;
						}
						break;
					case MSG_DONE:
						if (parameters.computing)
						{
							fprintf(stderr, "INFO: Nucleo sent a chunk %d\n",parameters.current_chunk);
							parameters.computing = false;
							if (parameters.current_chunk != img->number_of_chunks) // some chunks are left
							{
								event ev = {.source = EV_KEYBOARD, .type = EV_COMPUTE};
								queue_push(ev);
								fprintf(stderr, "INFO: Sending request for another chunk to nucleo\n");
							}
							else // no more chunks to calculate
							{
								parameters.computing = false;
								fprintf(stderr, "INFO: Nucleo has computed the image\r\n");
							}
						}
						else
							fprintf(stderr, "INFO: Abort from computer\r\n");
						break;
					case MSG_COMPUTE_DATA:
						if (parameters.current_chunk != msg->data.compute_data.cid)
								fprintf(stderr,"WARN: compute data chunk id %d is different from chunk id %d\x0a\r\n", msg->data.compute_data.cid, parameters.current_chunk);
							else
							{
								int x = (parameters.current_chunk % img->chunksinrow) * img->chunksize + msg->data.compute_data.i_re; 
								int y = parameters.current_chunk / img->chunksinrow * img->chunksize + msg->data.compute_data.i_im; 	
								if(!img)
								{
									fprintf(stderr,"ERROR: Not existing image \r\n");
									exit(101);
								}
								else if (x < 0 || x >= img->W || y < 0 || y >= img->H)
									fprintf(stderr,"WARN: pixel is out of bounds\r\n");
								else 
								{
									if (parameters.current_chunk < 0 || parameters.current_chunk >= img->number_of_chunks)
										fprintf (stderr, "ERROR: Invalid chunk id\n");
									else
									{
										double t = (double)msg->data.compute_data.iter / (double)parameters.n;
										img->data[y * img->W + (img->W - x - 1)] = (pix){ .r = (int8_t)get_red(t), .g = (uint8_t)get_green(t), .b = (uint8_t)get_blue(t)};
										xwin_redraw(img->W, img->H, (unsigned char*)img->data);
									}
								}
							}
						break;
					default:
						break;
				}
				if (msg) 
				{
					free(msg);
				}
			}
			else if (ev.type == EV_QUIT) 
			{
				quit = true;
			} 
			else {/*ignor*/}
		}
	
	}

    queue_cleanup(); // cleanup all events and free allocated memory for messages
	free(img->data);
	free(img);
    xwin_close();

   	for(int i = 0; i < NUM_THREADS; ++i) // joining threads
	{
		fprintf(stderr, "INFO: Call join to the thread %s\n", threads_names[i]);
		int r = pthread_join(threads[i], NULL);
		fprintf(stderr, "INFO: Joining the thread %s has been %s\n", threads_names[i], (r == 0 ? "OK" : "FAIL"));
   	}
	
	serial_close(data.fd);
	call_termios(1); // restore terminal settings
	return EXIT_SUCCESS;
}

// -- image window initialization-- //
image* init_image(int WIDTH, int HEIGHT,  int chunksize)
{
	image* img =(image*)malloc(sizeof(image));
	img->data = (pix*)malloc(WIDTH * HEIGHT * sizeof(pix)); // mallocs data for each pixel
	img->H = HEIGHT;
	img->W = WIDTH;
	img->refresh_requested = false;  // was just refreshed
	img->chunksinrow = (uint8_t)(WIDTH % chunksize == 0)?  WIDTH / chunksize : WIDTH / chunksize + 1; // chinks on line
	img->number_of_chunks = (uint8_t)((HEIGHT % chunksize == 0)? HEIGHT / chunksize : HEIGHT / chunksize + 1) * img->chunksinrow; // number of chunks in picture
	img->chunksize = chunksize;
	xwin_init(img->W, img->H); // inicializace okna
	img = dark_image(img); // nastaveni na tmavou
	return img;
}

// -- shows black image -- //
image* dark_image(image* img)
{
	for (int i = 0; i < img->H*img->W; i++) // each pixel (pixels are in vector)
		img->data[i] = (pix) {.r = 0,.g = 0, .b = 0};
	xwin_redraw(img->W, img->H, (unsigned char*)img->data);
	return img;
}

// -- gets chunk size -- //
uint8_t get_chunk_size (int width, int height)
{
	for (int i = DEFAUL_CHUNKSIZE; i < MAX_CHUNKSIZE; i += 20)
		if (i * i * 255 > width * height) return (uint8_t)i;
	return 250;
}

// -- computes data for image on PC -- //
image* compute_on_pc(image* img, computation parameters)
{
	for (int i = 0; i < img->H; i++)
	{
		for (int j = 0; j < img->W; j++)
		{
			double ZReal = parameters.aR + (img->W - j) * parameters.stepR;
			double ZImag = parameters.aI + i * parameters.stepI;
			double OldZReal, OldZImag;
			int iteration = 0;
			// computation for each pixel
			while (iteration < parameters.n && (ZReal*ZReal + ZImag*ZImag) < 4) // sqrt(2) = 4
			{
				OldZReal = ZReal;
				OldZImag = ZImag;
				ZReal = OldZReal * OldZReal - OldZImag * OldZImag + parameters.cR;
				ZImag = 2 * OldZReal * OldZImag + parameters.cI;
				iteration++;
			}
			// coloring according to number of iterations
			double t = (double)iteration / (double)parameters.n;
			img->data[i * img->W + j] = (pix){ .r = (int8_t)get_red(t), .g = (uint8_t)get_green(t), .b = (uint8_t)get_blue(t)};
		}
	}
	xwin_redraw(img->W,img->H, (unsigned char*)img->data); // redraw picture
	return img;
}

// -- creates multiple pictures (moved by par. C) and shows them to user -- //
image* pc_animation(image* img, computation parameters)
{
	for (int i = 0; i < 2800; i++)
	{ 
		compute_on_pc(img, parameters);
		parameters.cR += 0.0005;
		parameters.cI += 0.0005;
		xwin_poll_events();
	}
	return img;
}

// -- handels terminal settings -- //
void call_termios(int reset)
{
	static struct termios tio, tioOld;
	tcgetattr(STDIN_FILENO, &tio);
	if (reset) 
	{
		tcsetattr(STDIN_FILENO, TCSANOW, &tioOld);
	} else {
		tioOld = tio;
		cfmakeraw(&tio);
		tio.c_oflag |= OPOST;
		tcsetattr(STDIN_FILENO, TCSANOW, &tio);
	}
}

// -- reads comands from keyboard -- //
void* input_thread(void* d)
{
	data_t *data = (data_t*)d;
	bool end = false;
	char c;
	event ev;
	ev.source = EV_KEYBOARD;
	while ( !end && (c = getchar()))
	{
		ev.type = EV_TYPE_NUM;
		switch(c) 
		{
			case 'g':
				ev.type = EV_GET_VERSION;
				break;
			case 'c':  // parametr edit
				print_settings();
				c = getchar();
				ev.type = EV_CHANGE_PARAMETER;
				switch (c)
				{
					case 'r':
						ev.data.param = 0;
						break;
					case 'i':
						ev.data.param = 1;
						break;
					case 'w':
						ev.data.param = 2;
						break;
					case 'h':
						ev.data.param = 3;
						break;
					case 'n':
						ev.data.param = 4;
						break;
					default:
						ev.data.param = 5;
						break;
				}
				break;
			case '+': // after parameter edit expecting + or -
				ev.type = EV_CHANGE;
				ev.data.param = 1;
				break;
			case '-':
				ev.type = EV_CHANGE;
				ev.data.param = 0;
				break;
			case 'n':
				ev.type = EV_COMPUTE;
				break;
			case 'a':
				ev.type = EV_ABORT;
				break;
			case 'b':
				ev.type = EV_CLEAR_BUFFER;
				break;
			case '0':
				ev.type = EV_RESET_CHUNK;
				break;
			case 'r':
				ev.type = EV_REFRESH_IMAGE;
				break;
			case 'p':
				ev.type = EV_COMPUTE_PC;
				break;
			case 's':
				ev.type = EV_SET_COMPUTE;
				break;
			case 'm':
				ev.type = EV_SHOW_ANIMATION;
				break;
			case 'q': // EV_QUIT send later
				end = true;
				break;
			case 'h':
				print_help();
				break;
			default:
				fprintf(stderr, "WARN: Unkonown command\r\n");
				fprintf(stderr, "INFO: Press 'h' to see help\n");
				break;
		}
		if(ev.type != EV_TYPE_NUM)
			queue_push(ev);
		pthread_mutex_lock(&mtx);
		end = end || data->quit; 
		data->quit = end;
		pthread_mutex_unlock(&mtx);
   	}
	ev.type = EV_QUIT;
	queue_push(ev);
	fprintf(stderr, "INFO: Exit input thread %p\n", (void*)pthread_self());
	return NULL;
}

// -- shows how to use program -- //
void print_help()
{
	fprintf(stderr, ".__________________________________________.\n\r");
	fprintf(stderr, "|                                          |\r\n");
	fprintf(stderr, "|                   HELP                   |\r\n");
	fprintf(stderr, "|                                          |\r\n");
	fprintf(stderr, "|  s     Set new parameters                |\n\r");
	fprintf(stderr, "|  c     Change parameters                 |\n\r");
	fprintf(stderr, "|  n     Compute on Nucleo                 |\n\r");
	fprintf(stderr, "|  p     Compute on PC                     |\r\n");
	fprintf(stderr, "|  m     Movement (PC animation)           |\r\n");
	fprintf(stderr, "|  a     Abort computation                 |\r\n");
	fprintf(stderr, "|  r     Refresh image                     |\r\n");
	fprintf(stderr, "|  0     Reset chunk                       |\r\n");
	fprintf(stderr, "|  b     Reset buffer                      |\r\n");
	fprintf(stderr, "|  g     Get version                       |\r\n");
	fprintf(stderr, "|  q     Exit                              |\r\n");
	fprintf(stderr, "|__________________________________________|\r\n");
}

// -- shows how to set computation data -- //
void print_settings()
{
	fprintf(stderr,"._____________________________________.\r\n");
	fprintf(stderr,"|                                     |\n\r");
	fprintf(stderr,"|           PARAMETR SETTINGS         |\n\r");
	fprintf(stderr,"|                                     |\n\r");
	fprintf(stderr,"| Pick which parametr to change:      |\n\r");
	fprintf(stderr,"|  r     C REAL                       |\n\r");
	fprintf(stderr,"|  i     C IMAGINARY                  |\n\r");
	fprintf(stderr,"|  w     WIDTH                        |\n\r");
	fprintf(stderr,"|  h     HEIGHT                       |\n\r");
	fprintf(stderr,"|  n     Number of iterations         |\n\r");
	fprintf(stderr,"|_____________________________________|\n\r");
}

// -- shows how to set computation data -- //
void print_plus_minus()
{
	fprintf(stderr,"._____________________________________.\r\n");
	fprintf(stderr,"|                                     |\n\r");
	fprintf(stderr,"|          INCREASE X DECREASE        |\n\r");
	fprintf(stderr,"|                                     |\n\r");
	fprintf(stderr,"|  +     Increase value               |\n\r");
	fprintf(stderr,"|  -     Decrease value               |\n\r");
	fprintf(stderr,"|_____________________________________|\n\r");
}

// -- reads data from serial port -- //
void* serial_rx_thread(void* d)
{
		// read bytes from the serial and puts the parsed message to the queue
	data_t *data = (data_t*)d;
	uint8_t msg_buf[sizeof(message)]; // maximal buffer for all possible messages defined in messages.h
	event ev = { .source = EV_NUCLEO, .type = EV_SERIAL, .data.msg = NULL };
	bool end = false;
	unsigned char c;
	pthread_mutex_lock(&mtx);
	int fd = data->fd;
	pthread_mutex_unlock(&mtx);

	while (serial_getc_timeout(fd, SERIAL_READ_TIMOUT_MS, &c) > 0) {/* trash */}; 
	
	while (!end) 
	{
		pthread_mutex_lock(&mtx);
		fd = data->fd;
		pthread_mutex_unlock(&mtx);
		int r = serial_getc_timeout(fd, SERIAL_READ_TIMOUT_MS, &c);
		
		if (r > 0) // character has been read
		{ 
			if(get_message(c, data, msg_buf, &ev))
			{
				queue_push(ev);
			}
		}
		else if (r == 0) {/*read but nothing has been received*/ }
		else if (r>=MSG_NBR)
		{
			fprintf(stderr, "ERROR: Unknown message type has been received 0x%x\n - '%c'\r", c, c);
		}
		else
		{
			fprintf(stderr, "ERROR: Cannot receive data from the serial port\n");
			end = true;
		}
		pthread_mutex_lock(&mtx);
		end = end || data->quit;
		pthread_mutex_unlock(&mtx);
	}
	ev.type = EV_QUIT;
	queue_push(ev);
	fprintf(stderr, "INFO: Exit serial_rx_thread %p\n", (void*)pthread_self());
	return NULL;
}

// -- sends the massage as buffer -- //
bool send_message(data_t *data, message *msg) 
{   
   int MSG_SIZE = sizeof(message);
   uint8_t buffer[MSG_SIZE];
   int msg_length;
   if(!fill_message_buf(msg, buffer, MSG_SIZE, &msg_length))
   {
      fprintf(stderr, "ERROR: Buffer not fillled\n");
      return false;
   }
   pthread_mutex_lock(&mtx);
   int fd = data->fd;
   pthread_mutex_unlock(&mtx);
   return write(fd, buffer, msg_length);
}

// -- reads message of given type from serial input -- //
bool get_message(unsigned char msg_type, data_t *data, uint8_t *buffer, event *ev)
{
   // returns true if message was red correctly
      // puts message into event
   // returns false on error + prints error
      // free the message (unused)
   message *msg = malloc(sizeof(message));
   if(!msg)
   {
      fprintf(stderr, "Message failed to allocate\n");
      exit(1);
   }
   buffer[0] = msg_type;

   int msg_size;
   if(!get_message_size(msg_type, &msg_size))
   {
      fprintf(stderr, "ERROR: Message size not found, type %d\n", buffer[0]);
      free(msg);
      return false;
   }
   
   unsigned char c;
   pthread_mutex_lock(&mtx);
   int fd = data->fd;
   pthread_mutex_unlock(&mtx);
   for(int i = 1; i < msg_size; ++i)
   {
      int r = serial_getc_timeout(fd, SERIAL_READ_TIMOUT_MS, &c);
      if(r == -1)
      {
         fprintf(stderr, "ERROR: Cannot receive data from the serial port\n");
         return false;
      }
      buffer[i] = c;
   }
   if(!parse_message_buf(buffer, msg_size, msg))
   {
      fprintf(stderr,"ERROR: Cannot parse message type %d\n\r", buffer[0]);
      free(msg);
      return false;
   }
   ev->data.msg = msg;
   return true;
}

// -- computation of colors based on t -- //
double get_red(double t)
{
	return(9 * (1-t) * t * t *  t *255);
}
double get_green(double t)
{
	return(15 * (1-t) * (1-t) * t * t *255);
}
double get_blue(double t)
{
	return(8.5 * (1-t) * (1-t) * (1-t) * t *255);
}