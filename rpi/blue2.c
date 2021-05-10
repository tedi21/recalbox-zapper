#include <linux/uinput.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>
#include <string.h>
#include <glib.h>
#include <pthread.h>
#include <assert.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <SDL2/SDL_image.h>
#include "gattlib.h"
#define _USE_MATH_DEFINES
#include <math.h>

sig_atomic_t EXIT_REQUESTED = 0;
timer_t timer_id = 0;
const int EXPIRE_S = 15;
#define COMMAND_SIZE 30U
const int INIT_SEQUENCE = 1;
const int STAB_SEQUENCE = 2;
const int CALIBRATION_SEQUENCE = 3;
const int GAME_SEQUENCE = 4;
const double DEG_TO_RAD = M_PI / 180.0;

typedef struct node {
    char cmd[COMMAND_SIZE];
    struct node *next;
} node_t;

FILE* DEBUG = 0;
#define PRINT(f_, ...) fprintf(DEBUG, (f_), ##__VA_ARGS__);fflush(DEBUG);

static GMainLoop *m_main_loop = NULL;
static pthread_mutex_t m_cond_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t m_condition = PTHREAD_COND_INITIALIZER;
static int m_signaled = 0;
static pthread_mutex_t m_queue_mutex = PTHREAD_MUTEX_INITIALIZER;
static node_t* m_queue = NULL;
static int m_screen_width, m_screen_height;
static SDL_Rect m_text_rect;
static SDL_Rect m_spin_rect;
static SDL_Rect m_count_rect;
static TTF_Font* m_font = NULL;
static SDL_Surface* m_init_surface = NULL;
static SDL_Texture* m_init_message = NULL;
static SDL_Surface* m_stab_surface = NULL;
static SDL_Texture* m_stab_message = NULL;
static SDL_Surface* m_count_surface = NULL;
static SDL_Texture* m_count_texture = NULL;
static SDL_Surface* m_spin_surface = NULL;
static SDL_Texture* m_spin_texture = NULL;
static int m_calib_point = 0;
static double m_yaw[9], m_pitch[9], m_roll[9]; 
static double m_middle_x = 0.0, m_left = 0.0, m_right = 0.0;
static double m_middle_y = 0.0, m_up = 0.0, m_down = 0.0;
static double m_deg_to_pixel_x1 = 0.0, m_deg_to_pixel_x2 = 0.0, m_deg_to_pixel_y1 = 0.0, m_deg_to_pixel_y2 = 0.0;
static double m_range_x = 0.0, m_elevation_y = 0.0;
static gatt_connection_t* m_connection = NULL;

void enqueue(node_t **head, const void* data, size_t data_length) {
    node_t *new_node = malloc(sizeof(node_t));
    if (!new_node) return;
	memset(new_node->cmd, 0, COMMAND_SIZE);
	memcpy(new_node->cmd, data, data_length);
    new_node->next = *head;
    *head = new_node;
}

void dequeue(node_t **head, char cmd[COMMAND_SIZE]) {
    node_t *current, *prev = NULL;
    if (*head == NULL) return;
    current = *head;
    while (current->next != NULL) {
        prev = current;
        current = current->next;
    }
	strncpy(cmd, current->cmd, COMMAND_SIZE);
    free(current);
    if (prev)
        prev->next = NULL;
    else
        *head = NULL;
}

double average3(double val1, double val2, double val3) {
	double a = val1 + val2 + val3;
	return (a / 3.);
}

void wait_for_event() {
    pthread_mutex_lock(&m_cond_mutex);
    while (!m_signaled)
    {
        pthread_cond_wait(&m_condition, &m_cond_mutex);
    }
    m_signaled = 0;
    pthread_mutex_unlock(&m_cond_mutex);
}

void event() {
    pthread_mutex_lock(&m_cond_mutex);
    m_signaled = 1;
	pthread_cond_signal(&m_condition);
    pthread_mutex_unlock(&m_cond_mutex);
}

void signal_handler(int signum) {
	PRINT("Signal END\n");
    if (m_main_loop != NULL) {
    	g_main_loop_quit(m_main_loop);
    }
    EXIT_REQUESTED = 1;
	event();
}

void emit(int fd, int type, int code, int val) {
   struct input_event ie;

   ie.type = type;
   ie.code = code;
   ie.value = val;
   /* timestamp values below are ignored */
   ie.time.tv_sec = 0;
   ie.time.tv_usec = 0;

   write(fd, &ie, sizeof(ie));
}

int create_mouse() {
	int fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK);
	if (fd == -1) {
		PRINT("Fail to open input interface.\n");
	}
 	else {
		// enable synchronization
		int ret = ioctl(fd, UI_SET_EVBIT, EV_SYN);

		// enable mouse button left and absolute events
		ret = ioctl(fd, UI_SET_EVBIT, EV_KEY);
		ret = ioctl(fd, UI_SET_KEYBIT, BTN_LEFT);

		ret = ioctl(fd, UI_SET_EVBIT, EV_ABS);		
		ret = ioctl(fd, UI_SET_ABSBIT, ABS_X);
		ret = ioctl(fd, UI_SET_ABSBIT, ABS_Y);

#ifdef __x86_64__
		int version;
		ret = ioctl(fd, UI_GET_VERSION, &version);
		if (ret == 0 && version >= 5) {
			struct uinput_abs_setup abs_setup_x, abs_setup_y;
			memset(&abs_setup_x, 0, sizeof(abs_setup_x));
			abs_setup_x.code = ABS_X;
			abs_setup_x.absinfo.value = 0;
			abs_setup_x.absinfo.minimum = 0;
			abs_setup_x.absinfo.maximum = UINT16_MAX;
			abs_setup_x.absinfo.fuzz = 0;
			abs_setup_x.absinfo.flat = 0;
			ret = ioctl(fd, UI_ABS_SETUP, &abs_setup_x);

			memset(&abs_setup_y, 0, sizeof(abs_setup_y));
			abs_setup_y.code = ABS_Y;
			abs_setup_y.absinfo.value = 0;
			abs_setup_y.absinfo.minimum = 0;
			abs_setup_y.absinfo.maximum = UINT16_MAX;
			abs_setup_y.absinfo.fuzz = 0;
			abs_setup_y.absinfo.flat = 0;
			ret = ioctl(fd, UI_ABS_SETUP, &abs_setup_y);

			struct uinput_setup usetup;
			memset(&usetup, 0, sizeof(usetup));
			usetup.id.bustype = BUS_USB;
			usetup.id.vendor = 0x1234; /* sample vendor */
			usetup.id.product = 0x5678; /* sample product */
			strcpy(usetup.name, "Virtual mouse");
            usetup.id.version = 1;
            usetup.ff_effects_max = 0;
			ret = ioctl(fd, UI_DEV_SETUP, &usetup); 
		}
		else {
#endif
			struct uinput_user_dev uud;
			memset(&uud, 0, sizeof(uud));
			uud.id.bustype = BUS_USB;
			uud.id.vendor  = 0x1234;
			uud.id.product = 0x5678;
			snprintf(uud.name, UINPUT_MAX_NAME_SIZE, "Virtual mouse");
            uud.id.version = 1;
            uud.ff_effects_max = 0;
			uud.absmin[ABS_X] = 0;
			uud.absmax[ABS_X] = UINT16_MAX;
			uud.absfuzz[ABS_X] = 0;
			uud.absflat[ABS_X] = 0;
			uud.absmin[ABS_Y] = 0;
			uud.absmax[ABS_Y] = UINT16_MAX;
			uud.absfuzz[ABS_Y] = 0;
			uud.absflat[ABS_Y] = 0;
			write(fd, &uud, sizeof(uud));
#ifdef __x86_64__
		}
#endif

		ret = ioctl(fd, UI_DEV_CREATE);
		if (ret == -1) {
			PRINT("Fail to create mouse device.\n");
		}
	}
	return fd;
}

void release_device(int fd) {
	ioctl(fd, UI_DEV_DESTROY);
	close(fd);
}

void timer_handler( int sig, siginfo_t *si, void *uc ) {
    timer_t *tidp;
    tidp = si->si_value.sival_ptr;

    if ( *tidp == timer_id )
 	{
		PRINT("No signal\n");
		if (m_main_loop != NULL) {
			g_main_loop_quit(m_main_loop);
		}
	}
}

void arm_timer() {
    struct itimerspec       its;

	its.it_interval.tv_sec = 0;
	its.it_interval.tv_nsec = 0;
	its.it_value.tv_sec = EXPIRE_S;
	its.it_value.tv_nsec = 0;
	if (timer_settime(timer_id, 0, &its, NULL) != 0) {
		PRINT("Failed to set timer : %s.\n", strerror(errno));	
	}
}

void make_timer() {
    struct sigevent         te;
    struct sigaction        sa;
    int                     sig_no = SIGRTMIN;

    /* Set up signal handler. */
    sa.sa_flags = SA_SIGINFO;
    sa.sa_sigaction = timer_handler;
    sigemptyset(&sa.sa_mask);
    if (sigaction(sig_no, &sa, NULL) == 0)
	{
		/* Set and enable alarm */
		te.sigev_notify = SIGEV_SIGNAL;
		te.sigev_signo = sig_no;
		te.sigev_value.sival_ptr = &timer_id;
		if (timer_create(CLOCK_REALTIME, &te, &timer_id) == 0) {
			arm_timer();
		}
		else {
			PRINT("Failed to create timer.\n");	
		}
	}
	else {
        PRINT("Failed to setup timer.\n");
    }
}

void ihm_fill_circle(SDL_Renderer *surface, int cx, int cy, int radius, int lc, int border, Uint8 r, Uint8 g, Uint8 b, Uint8 a) {
    double dyo = 1;
	for (dyo = 1; dyo <= radius; dyo += 1.0)
	{
		double dxo = floor(sqrt((2.0 * radius * dyo) - (dyo * dyo)));
		SDL_SetRenderDrawColor(surface, r, g, b, a);
		if (dyo > border)
		{
			SDL_RenderDrawLine(surface, cx - dxo, cy + dyo - radius, cx - dxo + border, cy + dyo - radius);
			SDL_RenderDrawLine(surface, cx + dxo - border, cy + dyo - radius, cx + dxo, cy + dyo - radius);
			SDL_RenderDrawLine(surface, cx - dxo, cy - dyo + radius, cx - dxo + border, cy - dyo + radius);
			SDL_RenderDrawLine(surface, cx + dxo - border, cy - dyo + radius, cx + dxo, cy - dyo + radius);
		}
		else
		{
			SDL_RenderDrawLine(surface, cx - dxo, cy + dyo - radius, cx + dxo, cy + dyo - radius);
			SDL_RenderDrawLine(surface, cx - dxo, cy - dyo + radius, cx + dxo, cy - dyo + radius);
		}
	}
	double dc = 0;
	for (dc = 0; dc < border/2; dc += 1.0)
	{
		SDL_RenderDrawLine(surface, cx - dc, cy - lc, cx - dc, cy + lc);
		SDL_RenderDrawLine(surface, cx + dc, cy - lc, cx + dc, cy + lc);
		SDL_RenderDrawLine(surface, cx - lc, cy - dc, cx + lc, cy - dc);
		SDL_RenderDrawLine(surface, cx - lc, cy + dc, cx + lc, cy + dc);
	}
}

void ihm_get_event(int* running, char* point) {
	SDL_Event event;
	while (SDL_PollEvent(&event)) {
		if (event.type == SDL_QUIT) {
			*running = 0;
		}
		else if (event.type == SDL_KEYDOWN) {
			if ((event.key.keysym.sym >= SDLK_0) && 
				(event.key.keysym.sym <= SDLK_8))
			{
				*point = event.key.keysym.sym;
			}
		}
	}
}

void ihm_text_messages(SDL_Renderer* renderer, int mode) {
	const SDL_Color white = {255, 255, 255}; 
	const int message_width = 800;
	const int message_height = 100;
	m_font = TTF_OpenFont("Pervitina-Dex-FFP.ttf", 96);
	m_text_rect.x = (m_screen_width - message_width) / 2;
	m_text_rect.y = (m_screen_height - message_height) / 2;
	m_text_rect.w = message_width;
	m_text_rect.h = message_height;

	if (mode == INIT_SEQUENCE) {
		m_init_surface = TTF_RenderText_Solid(m_font, "Poser le pistolet pour l'initialisation", white);
		m_init_message = SDL_CreateTextureFromSurface(renderer, m_init_surface);
	}

	if (mode == STAB_SEQUENCE) {
		m_stab_surface = TTF_RenderText_Solid(m_font, "Orienter le pistolet en X, Y et Z", white);
		m_stab_message = SDL_CreateTextureFromSurface(renderer, m_stab_surface);
	}
}

void ihm_count(SDL_Renderer* renderer, int mode) {
	const int message_width = 80;
 	const int message_height = 100;
	m_count_rect.x = (m_screen_width - message_width) / 2;
	m_count_rect.y = (m_screen_height - message_height) / 2 + message_height;
	m_count_rect.w = message_width;
	m_count_rect.h = message_height;

	if (mode == INIT_SEQUENCE) {
    	m_count_surface = IMG_Load("countdown.png");
    	m_count_texture = SDL_CreateTextureFromSurface(renderer, m_count_surface);
	}
}

void ihm_spin(SDL_Renderer* renderer, int mode) {
	const int message_width = 100;
 	const int message_height = 100;
	m_spin_rect.x = (m_screen_width - message_width) / 2;
	m_spin_rect.y = (m_screen_height - message_height) / 2 + message_height;
	m_spin_rect.w = message_width;
	m_spin_rect.h = message_height;

	if (mode == STAB_SEQUENCE) {
    	m_spin_surface = IMG_Load("circles.png");
    	m_spin_texture = SDL_CreateTextureFromSurface(renderer, m_spin_surface);
	}
}

void ihm_clean() {
	if (m_spin_texture) {
		SDL_DestroyTexture(m_spin_texture);
		m_spin_texture = NULL;
	}
	if (m_spin_surface) {
		SDL_FreeSurface(m_spin_surface);
		m_spin_surface = NULL;
	}
	if (m_count_texture) {
		SDL_DestroyTexture(m_count_texture);
		m_count_texture = NULL;
	}
	if (m_count_surface) {
		SDL_FreeSurface(m_count_surface);
		m_count_surface = NULL;
	}
	if (m_stab_message) {
		SDL_DestroyTexture(m_stab_message);
		m_stab_message = NULL;
	}
	if (m_stab_surface) {
		SDL_FreeSurface(m_stab_surface);
		m_stab_surface = NULL;
	}
	if (m_init_message) { 
		SDL_DestroyTexture(m_init_message);
		m_init_message = NULL;
	}
	if (m_init_surface) {
		SDL_FreeSurface(m_init_surface);
		m_init_surface = NULL;
	}
	if (m_font) {
		TTF_CloseFont(m_font);
		m_font = NULL;
	}
}

void ihm_init_sequence(SDL_Renderer* renderer, Uint32 ticks) {
	Uint32 seconds = ((ticks / 1000) % 11) + 5;
	SDL_Rect srcrect = { seconds * 96, 0, 96, 96 };
	SDL_RenderCopy(renderer, m_count_texture, &srcrect, &m_count_rect);
	SDL_RenderCopy(renderer, m_init_message, NULL, &m_text_rect);
}

void ihm_stab_sequence(SDL_Renderer* renderer, Uint32 ticks) {
	Uint32 frames = (ticks / 100) % 16;
	Uint32 range = frames / 4;
	Uint32 column = frames % 4;
	SDL_Rect srcrect = { column * 400, range * 400, 400, 400 };
	SDL_RenderCopy(renderer, m_spin_texture, &srcrect, &m_spin_rect);
	SDL_RenderCopy(renderer, m_stab_message, NULL, &m_text_rect);		
}

void ihm_calibration_sequence(SDL_Renderer* renderer, char point) {
	if (point == SDLK_0) {
		ihm_fill_circle(renderer, 30, 30, 25, 10, 2, 255, 255, 255, 255);
	}
	else if (point == SDLK_1) {
		ihm_fill_circle(renderer, 30, m_screen_height/2, 25, 10, 2, 255, 255, 255, 255);
	}
	else if (point == SDLK_2) {
		ihm_fill_circle(renderer, 30, m_screen_height - 30, 25, 10, 2, 255, 255, 255, 255);
	}
	else if (point == SDLK_3) {
		ihm_fill_circle(renderer, m_screen_width/2, m_screen_height - 30, 25, 10, 2, 255, 255, 255, 255);
	}
	else if (point == SDLK_4) {
		ihm_fill_circle(renderer, m_screen_width/2, m_screen_height/2, 25, 10, 2, 255, 255, 255, 255);
	}
	else if (point == SDLK_5) {
		ihm_fill_circle(renderer, m_screen_width/2, 30, 25, 10, 2, 255, 255, 255, 255);
	}
	else if (point == SDLK_6) {
		ihm_fill_circle(renderer, m_screen_width - 30, 30, 25, 10, 2, 255, 255, 255, 255);
	}
	else if (point == SDLK_7) {
		ihm_fill_circle(renderer, m_screen_width - 30, m_screen_height/2, 25, 10, 2, 255, 255, 255, 255);
	}
	else {
		ihm_fill_circle(renderer, m_screen_width - 30, m_screen_height - 30, 25, 10, 2, 255, 255, 255, 255);
	}
}

void* ihm_loop(void* arg) {
	const int mode = *((int*)arg);
	PRINT("MODE %d\n", mode);

	// Catch CTRL-C
	signal(SIGINT, signal_handler);

	const uint32_t WindowFlags = SDL_WINDOW_FULLSCREEN_DESKTOP;
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        PRINT("Unable to initialize SDL: %s", SDL_GetError());
    }
    if (TTF_Init() != 0) {
        PRINT("Unable to initialize TTF: %s", TTF_GetError());
    }
	SDL_Window *window = SDL_CreateWindow("Window", 0, 0, 0, 0, WindowFlags);
	SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
	SDL_GetRendererOutputSize(renderer, &m_screen_width, &m_screen_height);

	ihm_text_messages(renderer, mode);
	ihm_count(renderer, mode);
	ihm_spin(renderer, mode);

	int running = 1;
	char point = SDLK_0;
	while (running) {
		ihm_get_event(&running, &point);

		if (running) {
			SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
			SDL_RenderClear(renderer);
			Uint32 ticks = SDL_GetTicks();

			if (mode == INIT_SEQUENCE) {
				ihm_init_sequence(renderer, ticks);
			}
			else if (mode == STAB_SEQUENCE) {
				ihm_stab_sequence(renderer, ticks);
			}
			else if (mode == CALIBRATION_SEQUENCE) {
				ihm_calibration_sequence(renderer, point);
			}
			//Update screen
			SDL_RenderPresent(renderer);
		}
	}

	ihm_clean();
	SDL_DestroyRenderer(renderer);
	SDL_DestroyWindow(window);

	TTF_Quit();
	SDL_Quit();

	pthread_exit(NULL);
}

void ihm_quit() {
	SDL_Event event;
	event.type = SDL_QUIT;
	SDL_PushEvent(&event);
}

void ble_write(char value_data) {
	const uuid_t write_uuid = CREATE_UUID16(0xFFE1);
	gattlib_write_char_by_uuid(m_connection, &write_uuid, &value_data, sizeof(value_data));
}

void init_sequence(pthread_t* thread_ihm, int* mode) {
	// Create IHM thread
	pthread_create(thread_ihm, NULL, ihm_loop, mode);
	sleep(5);
	ihm_quit();
	pthread_join(*thread_ihm, NULL);

	ble_write('Z');

	PRINT("Wait for gyrometer stabilization\n");
	*mode = STAB_SEQUENCE;
	pthread_create(thread_ihm, NULL, ihm_loop, mode);
}

void stab_sequence(char cmd[COMMAND_SIZE], pthread_t* thread_ihm, int* mode) {
	ble_write('Y');

	PRINT("Stabilization OK\n");
	ihm_quit();
	pthread_join(*thread_ihm, NULL);

	PRINT("Calibration\n");
	*mode = CALIBRATION_SEQUENCE;
	m_calib_point = 0;
	pthread_create(thread_ihm, NULL, ihm_loop, mode);
}

void calibration_sequence(char cmd[COMMAND_SIZE], pthread_t* thread_ihm, int* mode) {
	if (m_calib_point < 9) {	
		double yaw = 0.0, pitch = 0.0, roll = 0.0;
		sscanf(cmd + 1, "%lf %lf %lf", &yaw, &pitch, &roll);
		m_yaw[m_calib_point] = yaw;
		m_pitch[m_calib_point] = pitch;
		m_roll[m_calib_point] = roll;

		if (m_calib_point == 8) {
			ble_write('X');

			m_middle_x = average3(m_yaw[3], m_yaw[4], m_yaw[5]);
			m_left = average3(m_yaw[0], m_yaw[1], m_yaw[2]);
			m_right = average3(m_yaw[6], m_yaw[7], m_yaw[8]);
			m_up = average3(m_pitch[0], m_pitch[5], m_pitch[6]);
			m_middle_y = average3(m_pitch[1], m_pitch[4], m_pitch[7]);
			m_down = average3(m_pitch[2], m_pitch[3], m_pitch[8]);

			m_deg_to_pixel_x1 = (m_screen_width/2 - 30) / (m_middle_x - m_left);
			m_deg_to_pixel_x2 = (m_screen_width/2 - 30) / (m_right - m_middle_x);
			m_deg_to_pixel_y1 = (m_screen_height/2 - 30) / (m_middle_y - m_down);
			m_deg_to_pixel_y2 = (m_screen_height/2 - 30) / (m_up - m_middle_y);

			PRINT("Parameters\n");
			PRINT("m_screen_width : %d\n", m_screen_width);
			PRINT("m_screen_height : %d\n", m_screen_height);
			PRINT("m_middle_x : %lf\n", m_middle_x);
			PRINT("m_middle_y : %lf\n", m_middle_y);
			PRINT("m_left : %lf\n", m_left);
			PRINT("m_right : %lf\n", m_right);
			PRINT("m_up : %lf\n", m_up);
			PRINT("m_down : %lf\n", m_down);
			PRINT("m_deg_to_pixel_x1 : %lf\n", m_deg_to_pixel_x1);
			PRINT("m_deg_to_pixel_x2 : %lf\n", m_deg_to_pixel_x2);
			PRINT("m_deg_to_pixel_y1 : %lf\n", m_deg_to_pixel_y1);
			PRINT("m_deg_to_pixel_y2 : %lf\n", m_deg_to_pixel_y2);

			m_calib_point = 0;
			PRINT("Calibration OK\n");
			ihm_quit();
			pthread_join(*thread_ihm, NULL);

			PRINT("Game\n");
			*mode = GAME_SEQUENCE;
		}
		else {
			++m_calib_point;
			SDL_Event sdlevent;
			sdlevent.type = SDL_KEYDOWN;
			sdlevent.key.keysym.sym = SDLK_0 + m_calib_point;
			SDL_PushEvent(&sdlevent);
		}
	}
	else {
		m_calib_point = 0;
	}
}

void angle_to_screen(double yaw, double pitch, double roll, int* x, int* y) {
	double pixel_x = 0.0;
	double pixel_y = 0.0;
	if (yaw < m_middle_x) {
		pixel_x = m_screen_width/2 - (m_middle_x - yaw) * m_deg_to_pixel_x1;
	}
	else {
		pixel_x = m_screen_width/2 + (yaw - m_middle_x) * m_deg_to_pixel_x2;
	}
	if (pitch < m_middle_y) {
		pixel_y = m_screen_height/2 + (m_middle_y - pitch) * m_deg_to_pixel_y1;
	}
	else {
		pixel_y = m_screen_height/2 - (pitch - m_middle_y) * m_deg_to_pixel_y2;
	}

    *x = (pixel_x / m_screen_width) * UINT16_MAX;
	if (*x < 0) *x = 0;
	if (*x > UINT16_MAX) *x = UINT16_MAX;

    *y = (pixel_y / m_screen_height) * UINT16_MAX;
	if (*y < 0) *y = 0;
	if (*y > UINT16_MAX) *y = UINT16_MAX;
}

void game_sequence(char cmd[COMMAND_SIZE], int fd) {
	double yaw = 0.0, pitch = 0.0, roll = 0.0;
	sscanf(cmd + 1, "%lf %lf %lf", &yaw, &pitch, &roll);

	int x = 0, y = 0;
	angle_to_screen(yaw, pitch, roll, &x, &y);

	emit(fd, EV_KEY, BTN_LEFT, 1);
	emit(fd, EV_ABS, ABS_X, x);
	emit(fd, EV_ABS, ABS_Y, y);
	emit(fd, EV_SYN, SYN_REPORT, 0);
	usleep(20000);
	emit(fd, EV_KEY, BTN_LEFT, 0);
	emit(fd, EV_ABS, ABS_X, x);
	emit(fd, EV_ABS, ABS_Y, y);
	emit(fd, EV_SYN, SYN_REPORT, 0);
}

void aim_sequence(char cmd[COMMAND_SIZE], int fd) {
	double yaw = 0.0, pitch = 0.0, roll = 0.0;
	sscanf(cmd + 1, "%lf %lf %lf", &yaw, &pitch, &roll);

	int x = 0, y = 0;
	angle_to_screen(yaw, pitch, roll, &x, &y);

	emit(fd, EV_ABS, ABS_X, x);
	emit(fd, EV_ABS, ABS_Y, y);
	emit(fd, EV_SYN, SYN_REPORT, 0);
}

void* route_message(void* arg) {
	const int fd = *((int*)arg);
	char cmd[COMMAND_SIZE];
	int mode = 0;
	pthread_t thread_ihm;

	// Catch CTRL-C
	signal(SIGINT, signal_handler);

	PRINT("Start route message\n");
	while(!EXIT_REQUESTED) {
		cmd[0] = '\0';
		wait_for_event();
		while (m_queue != NULL) {
			pthread_mutex_lock(&m_queue_mutex);
			dequeue(&m_queue, cmd);
			pthread_mutex_unlock(&m_queue_mutex);

			//PRINT("Message %s\n", cmd);

			char id = cmd[0];
			if (id == 'A') {
				if ((mode != 0) && (mode != GAME_SEQUENCE)) {
					ihm_quit();
					PRINT("Wait IHM\n");
					pthread_join(thread_ihm, NULL);
				}
				// Start initialization sequence 
				PRINT("Start initialization sequence\n");
				mode = INIT_SEQUENCE;
				init_sequence(&thread_ihm, &mode);			
			}
			else if (id == 'B' && mode == STAB_SEQUENCE) {
				PRINT("Command %s\n", cmd);
				// Wait for gyrometer stabilization
				stab_sequence(cmd, &thread_ihm, &mode);	
			}
			else if (id == 'C' && mode == CALIBRATION_SEQUENCE) {
				PRINT("Command %s\n", cmd);
				// Calibration
				calibration_sequence(cmd, &thread_ihm, &mode);	
			}
			else if (id == 'D' && mode == GAME_SEQUENCE) {
				PRINT("Command %s\n", cmd);
				game_sequence(cmd, fd);
			}
			else if (id == 'E' && mode == GAME_SEQUENCE) {
				aim_sequence(cmd, fd);
			}
		}
	}

	if ((mode != 0) && (mode != GAME_SEQUENCE)) {
		ihm_quit();
		PRINT("Wait IHM\n");
		pthread_join(thread_ihm, NULL);
	}
	PRINT("End route\n");
	pthread_exit(NULL);
}

void ble_notification_cb(uint16_t handle, const uint8_t* data, size_t data_length, void* user_data) {
	static char buffer[COMMAND_SIZE] = {0, 0, 0};
	static char buf_length = 0;
	if (data != NULL && data_length > 0) {		
		if (buf_length + data_length < COMMAND_SIZE) {
			memcpy(buffer + buf_length, data, data_length);
			buf_length += data_length;
			if ((buf_length > 0) && (buffer[buf_length - 1] == ';')) {
				// Route message
				pthread_mutex_lock(&m_queue_mutex);
				enqueue(&m_queue, buffer, buf_length);
				pthread_mutex_unlock(&m_queue_mutex);
				event();
				//PRINT("Notification %s\n", buffer);
				memset(buffer, 0, COMMAND_SIZE);
				buf_length = 0;
				arm_timer();
			}
		}
		else {
			buf_length = 0;
		}
	}
}


int main(void) {
	DEBUG = fopen("/recalbox/share/scripts/log.txt","w");

	// Create virtual mouse
	int fd = create_mouse();

	// Catch CTRL-C
	signal(SIGINT, signal_handler);

	// Create router thread
	pthread_t thread_router;
	pthread_create(&thread_router, NULL, route_message, &fd);

	while (!EXIT_REQUESTED) {
		// Connect to the bluetooth liaison
		m_connection = gattlib_connect(NULL, "3C:A5:08:0A:62:A9", GATTLIB_CONNECTION_OPTIONS_LEGACY_DEFAULT);
		if (m_connection == NULL) {
			PRINT("Fail to connect to the bluetooth device.\n");
		}
		else {
			const uuid_t ble_input_uuid = CREATE_UUID16(0xFFE1);
			const uint16_t enable_notification = 0x0001;
			// Enable Status Notification
			int ret = gattlib_write_char_by_uuid(m_connection, &ble_input_uuid, &enable_notification, sizeof(enable_notification));
			if (ret == GATTLIB_SUCCESS) {

				gattlib_register_notification(m_connection, ble_notification_cb, NULL);

				ret = gattlib_notification_start(m_connection, &ble_input_uuid);
				if (ret == GATTLIB_SUCCESS) {

					// Start keep alive
					make_timer();

					m_main_loop = g_main_loop_new(NULL, 0);
					g_main_loop_run(m_main_loop);

					PRINT("Disconnection.\n");

					// In case we quit the main loop, clean the connection
					gattlib_notification_stop(m_connection, &ble_input_uuid);
					g_main_loop_unref(m_main_loop);
				}
				else {
					PRINT("Fail to start notification\n.");
				}
			}
			else {
				PRINT("Fail to enable notification\n.");
			}

			gattlib_disconnect(m_connection);
			m_connection = NULL;
		}
		if (!EXIT_REQUESTED) {
			// retry 5 seconds later
			sleep(5);
		}
	}

	PRINT("Wait router\n");
	pthread_join(thread_router, NULL);

	if (fd != -1) {
		release_device(fd);
	}
	PRINT("Bye\n");
	fclose(DEBUG);
	return 0;
}
