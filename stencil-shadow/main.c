#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glx.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>

//#define DEBUG TRUE
#define FPS_COUNTER TRUE

#define DEF_WIDTH  640
#define DEF_HEIGHT 480
#define FPS_COUNTER_NUM_FRAMES 1000

#define FALSE 0
#define TRUE  1

#define MODE_NORMAL  0
#define MODE_SHADOW  1
#define MODE_REFLECT 2

struct {
	Display *disp;
	int screen;
	Window win;
	int fullscreen;
	GLXContext ctx;

	int w, h;
	int win_w, win_h;
	unsigned int depth;
	int doublebuf;

	GLuint tex_ground;
	GLuint tex_model;
	int texturing;
	int color_texture;
	int lighting;
	int shadowing;
	int reflection;

	GLfloat light_pos[4];
} appdata;

void event_loop(void);
void init_context(void);
void toggle_fullscreen(void);
void load_textures(void);
void mult_by_shadow_matrix(GLfloat plane[4], GLfloat light[4]);
void draw_model(int mode);
void draw_plane(int mode);
void init_gl(void);
void resize_scene(void);
void draw_stuff(double t, double angle, double pitch);

int main(int argc, char **argv) {
	XEvent event;
	unsigned long black;

	appdata.disp = XOpenDisplay(NULL);
	if(appdata.disp == NULL) {
		fprintf(stderr, "ERROR: Couldn't open X display!\n");
		exit(1);
	}

	appdata.screen = XDefaultScreen(appdata.disp);

	appdata.win_w = DEF_WIDTH;
	appdata.win_h = DEF_HEIGHT;
	appdata.fullscreen = FALSE;
	appdata.texturing = TRUE;
	appdata.color_texture = FALSE;
	appdata.lighting = TRUE;
	appdata.shadowing = TRUE;
	appdata.reflection = TRUE;
	appdata.light_pos[0] = 0.0;
	appdata.light_pos[1] = 1.0;
	appdata.light_pos[2] = 0.0;
	appdata.light_pos[3] = 0.0;

	black = XBlackPixel(appdata.disp, appdata.screen);

	init_context();

	event_loop();

	XCloseDisplay(appdata.disp);

	return 0;
}

void event_loop(void) {
	int quitting = FALSE;
	int paused = TRUE;
	int drawing_pause = TRUE;
	Atom wm_delete;
	XEvent event;
	KeySym ksym;
	double t = 0.0, t_speed = 0.0;
	int angle_rotation = 0;  /* Key pressed */
	double angle = 0.0, angle_speed = 0.0;
	int pitch_rotation = 0;
	double pitch = 30.0, pitch_speed = 0.0;
	int mouse_x, mouse_y;
	int day_progression = FALSE;
	double day_phase = 0.0;
#if FPS_COUNTER
	struct timespec ts;
	int frames = 0;
	double start_time, end_time;
#endif

	wm_delete = XInternAtom(appdata.disp, "WM_DELETE_WINDOW", TRUE);

#if DEBUG
	printf("\n * Entering main loop\n");
	printf(  " * ------------------\n");
#endif

#if FPS_COUNTER
#if DEBUG
	clock_getres(CLOCK_MONOTONIC, &ts);
	printf(" * CLOCK_MONOTONIC resolution: %ld ns\n",
	 1000000000*ts.tv_sec + ts.tv_nsec);
#endif
	clock_gettime(CLOCK_MONOTONIC, &ts);
	start_time = ts.tv_sec + 1.0e-9*ts.tv_nsec;
#endif

	while(!quitting) {
		while(XPending(appdata.disp)) {
			XNextEvent(appdata.disp, &event);
			switch(event.type) {
			case KeyPress:
				ksym = XKeycodeToKeysym(appdata.disp, event.xkey.keycode, 0);
				switch(ksym) {
				case XK_Left:
					angle_rotation = 1;
					break;
				case XK_Right:
					angle_rotation = -1;
					break;
				case XK_Up:
					pitch_rotation = 1;
					break;
				case XK_Down:
					pitch_rotation = -1;
					break;
				case XK_space:
					paused = !paused;
					printf("%s\n",
					 paused ? "Paused" : "Unpaused");
					break;
				case XK_f:
					drawing_pause = !drawing_pause;
					printf("Super Turbo Mode %sabled.\n",
					 !drawing_pause ? "en" : "dis");
					break;
				case XK_t:
					appdata.texturing = !appdata.texturing;
					printf("Texturing %sabled.\n",
					 appdata.texturing ? "en" : "dis");
					break;
				case XK_c:
					appdata.color_texture = !appdata.color_texture;
					printf("Coloring %sabled.\n",
					 appdata.color_texture ? "en" : "dis");
					break;
				case XK_l:
					appdata.lighting = !appdata.lighting;
					printf("Lighting %sabled.\n",
					 appdata.lighting ? "en" : "dis");
					break;
				case XK_s:
					appdata.shadowing = !appdata.shadowing;
					printf("Shadow %sabled.\n",
					 appdata.shadowing ? "en" : "dis");
					break;
				case XK_r:
					appdata.reflection = !appdata.reflection;
					printf("Reflection %sabled.\n",
					 appdata.reflection ? "en" : "dis");
					break;
				case XK_d:
					day_progression = !day_progression;
					printf("Day cycle %sabled.\n",
					 day_progression ? "en" : "dis");
					break;
				case XK_F11:
					toggle_fullscreen();
					break;
				case XK_h:
					printf("Commands:\n"
					 "  Left/Right:  Rotate\n"
					 "  Space:       Pause animation\n"
					 "  F:           Super Turbo Mode\n"
					 "  T:           Texturing\n"
					 "  C:           Coloring\n"
					 "  L:           Lighting\n"
					 "  S:           Shadow\n"
					 "  R:           Reflection\n"
					 "  D:           Day cycle\n"
					 "  F11:         Fullscreen\n"
					 "  Escape/Q:    Quit program\n");
					break;
				case XK_q:
				case XK_Escape:
					quitting = TRUE;
					break;
				default:
#if DEBUG
					printf(" * Unhandled key: \"%s\" (%d)\n",
					 XKeysymToString(ksym), (int)ksym);
#endif
					break;
				}
				break;
			case KeyRelease:
				ksym = XKeycodeToKeysym(appdata.disp, event.xkey.keycode, 0);
				switch(ksym) {
				case XK_Left:
				case XK_Right:
					angle_rotation = 0;
					break;
				case XK_Up:
				case XK_Down:
					pitch_rotation = 0;
					break;
				}
				break;
			case ButtonPress:
				if(event.xbutton.button == Button1) {
					mouse_x = event.xbutton.x;
					mouse_y = event.xbutton.y;
				}
				break;
			case MotionNotify:
				if(event.xmotion.state & Button1Mask) {
					angle += (event.xmotion.x-mouse_x)*0.2;
					pitch += (event.xmotion.y-mouse_y)*0.2;
					XWarpPointer(appdata.disp, None, appdata.win, 0,0,0,0,
					 mouse_x,mouse_y);
				}
				break;
			case ConfigureNotify:
				if(event.xconfigure.width != appdata.w ||
				 event.xconfigure.height != appdata.h) {
					appdata.w = event.xconfigure.width;
					appdata.h = event.xconfigure.height;
					resize_scene();
#if DEBUG
					printf(" * New resolution: %dx%d\n", appdata.w, appdata.h);
#endif
				}
				break;
			case ClientMessage:
				if(event.xclient.data.l[0] == wm_delete) {
					quitting = TRUE;
				}
				break;
			default:
#if DEBUG
				printf(" * Unhandled event: type = %d\n", event.type);
#endif
				break;
			}
		}
		angle_speed = (angle_speed+0.05*angle_rotation)*0.93;
		angle += angle_speed;
		if(angle >=  360.0) angle -= 360.0;
		if(angle <= -360.0) angle += 360.0;

		pitch_speed = (pitch_speed+0.05*pitch_rotation)*0.93;
		pitch += pitch_speed;
		if(pitch >= 90.0) {
			pitch = 90.0;
			pitch_speed = 0.0;
		}
		if(pitch <= -90.0) {
			pitch = -90.0;
			pitch_speed = 0.0;
		}

		appdata.light_pos[0] = -sin(day_phase);
		appdata.light_pos[1] = cos(day_phase);
		if(day_progression) day_phase += 0.005;
		if(day_phase >= 2*M_PI) day_phase -= 2*M_PI;

		draw_stuff(t, angle, pitch);

		t_speed = (t_speed+0.0002*(!paused))*0.9;
		t += t_speed;
		if(t >= 1.0) t -= 1.0;
		if(drawing_pause) {
			usleep(10000);
		}
#if FPS_COUNTER
		frames ++;
		if(frames >= FPS_COUNTER_NUM_FRAMES) {
			clock_gettime(CLOCK_MONOTONIC, &ts);
			end_time = ts.tv_sec + 1.0e-9*ts.tv_nsec;

			printf("%d frames in %6.3f s --> %6.0f FPS\n",
			 FPS_COUNTER_NUM_FRAMES, end_time-start_time,
			 FPS_COUNTER_NUM_FRAMES/(end_time-start_time));

			start_time = end_time;
			frames = 0;
		}
#endif
	}
}

void init_context(void) {
	int i;
	int ver_major, ver_minor;
	XVisualInfo *vi;
	int attrListDbl[] = { GLX_RGBA, GLX_DOUBLEBUFFER,
	 GLX_RED_SIZE, 4,
	 GLX_GREEN_SIZE, 4,
	 GLX_BLUE_SIZE, 4,
	 GLX_DEPTH_SIZE, 16,
	 GLX_STENCIL_SIZE, 1,
	 None };
	int attrListSgl[] = { GLX_RGBA,
	 GLX_RED_SIZE, 4,
	 GLX_GREEN_SIZE, 4,
	 GLX_BLUE_SIZE, 4,
	 GLX_DEPTH_SIZE, 16,
	 GLX_STENCIL_SIZE, 1,
	 None };
	XSetWindowAttributes swa;
	Atom wm_delete;
	Window dummy_win;
	int dummy_int;

#if DEBUG
	printf("\n * Initializing GLX\n");
	printf(  " * ----------------\n");
	glXQueryVersion(appdata.disp, &ver_major, &ver_minor);
	printf(" * GLX Version: %d.%d\n", ver_major, ver_minor);
#endif

	vi = glXChooseVisual(appdata.disp, appdata.screen, attrListDbl);
	if(vi == NULL) {
		vi = glXChooseVisual(appdata.disp, appdata.screen, attrListSgl);
		if(vi == NULL) {
			fprintf(stderr, "ERROR: Couldn't get GLX Visual\n");
			exit(1);
		}
#if DEBUG
		printf(" * Not using double-buffered visual\n");
	} else {
		printf(" * Using double-buffered visual\n");
#endif
	}

	 /* Create window: */
	swa.colormap = XCreateColormap(appdata.disp,
	 XRootWindow(appdata.disp, vi->screen), vi->visual, AllocNone);
	swa.border_pixel = 0;
	swa.event_mask = StructureNotifyMask|KeyPressMask|KeyReleaseMask|
	 ButtonPressMask|PointerMotionMask;
	if(appdata.fullscreen) {
		appdata.win_w = appdata.w;
		appdata.win_h = appdata.h;

		appdata.w = XWidthOfScreen(XScreenOfDisplay(appdata.disp,
		 appdata.screen));
		appdata.h = XHeightOfScreen(XScreenOfDisplay(appdata.disp,
		 appdata.screen));
		swa.override_redirect = True;
	} else {
		appdata.w = appdata.win_w;
		appdata.h = appdata.win_h;
		swa.override_redirect = False;
	}
	appdata.win = XCreateWindow(appdata.disp,
	 XRootWindow(appdata.disp, vi->screen), 0,0, appdata.w,appdata.h,
	 0, vi->depth, InputOutput, vi->visual,
	 CWBorderPixel|CWColormap|CWEventMask|CWOverrideRedirect, &swa);
	wm_delete = XInternAtom(appdata.disp, "WM_DELETE_WINDOW", TRUE);
	XSetWMProtocols(appdata.disp, appdata.win, &wm_delete, 1);
	XStoreName(appdata.disp, appdata.win, "GLX Demo");
	XMapRaised(appdata.disp, appdata.win);
	if(appdata.fullscreen) {
		XGrabKeyboard(appdata.disp, appdata.win, True, GrabModeAsync,
		 GrabModeAsync, CurrentTime);
		XGrabPointer(appdata.disp, appdata.win, True, ButtonPressMask,
		 GrabModeAsync, GrabModeAsync, appdata.win, None, CurrentTime);
	}

	XGetGeometry(appdata.disp, appdata.win, &dummy_win, &dummy_int, &dummy_int,
	 &appdata.w, &appdata.h, &dummy_int, &appdata.depth);
#if DEBUG
	printf(" * Using resolution %dx%d\n", appdata.w, appdata.h);
	printf(" * Using depth %d\n", appdata.depth);
#endif

	 /* Create context: */
	appdata.ctx = glXCreateContext(appdata.disp, vi, 0, True);
	glXMakeCurrent(appdata.disp, appdata.win, appdata.ctx);

	if(glXIsDirect(appdata.disp, appdata.ctx)) {
		appdata.doublebuf = TRUE;
#if DEBUG
		printf(" * Using direct rendering\n");
#endif
	} else {
		appdata.doublebuf = FALSE;
#if DEBUG
		printf(" * Not using direct rendering\n");
#endif
	}

	init_gl();
	resize_scene();

	XFree(vi);
}

void toggle_fullscreen(void) {
	 /* Destroy window: */
	glXMakeCurrent(appdata.disp, None, NULL);
	glXDestroyContext(appdata.disp, appdata.ctx);
	XDestroyWindow(appdata.disp, appdata.win);

	appdata.fullscreen = !appdata.fullscreen;

	init_context();
}

void load_textures(void) {
	FILE *infile;
	unsigned char *buffer;
	const int w=64, h=64;
//	const int bufsize = w*h;
	const int bufsize = w*h*3;

	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

	 /* Ground Texture: */
	infile = fopen("ground.raw", "r");
	if(infile == NULL) {
		fprintf(stderr, "ERROR: ");
		perror("fopen");
		exit(1);
	}
	buffer = (unsigned char *) malloc(bufsize);
	if(fread(buffer, 1, bufsize, infile) < bufsize) {
		fprintf(stderr, "ERROR: Short read on ground.raw\n");
		exit(1);
	}
	fclose(infile);

	glGenTextures(1, &appdata.tex_ground);
	glBindTexture(GL_TEXTURE_2D, appdata.tex_ground);

	/*
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	*/
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);

	/*
	glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, w,h, 0, GL_LUMINANCE,
	 GL_UNSIGNED_BYTE, buffer);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w,h, 0, GL_RGB,
	 GL_UNSIGNED_BYTE, buffer);
	*/
	gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGB, w,h, GL_RGB, GL_UNSIGNED_BYTE,
	 buffer);

	free(buffer);

	 /* Model Texture: */
	infile = fopen("model.raw", "r");
	if(infile == NULL) {
		fprintf(stderr, "ERROR: ");
		perror("fopen");
		exit(1);
	}
	buffer = (unsigned char *) malloc(bufsize);
	if(fread(buffer, 1, bufsize, infile) < bufsize) {
		fprintf(stderr, "ERROR: Short read on model.raw\n");
		exit(1);
	}
	fclose(infile);

	glGenTextures(1, &appdata.tex_model);
	glBindTexture(GL_TEXTURE_2D, appdata.tex_model);

	/*
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	*/
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);

	/*
	glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, w,h, 0, GL_LUMINANCE,
	 GL_UNSIGNED_BYTE, buffer);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w,h, 0, GL_RGB,
	 GL_UNSIGNED_BYTE, buffer);
	*/
	gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGB, w,h, GL_RGB, GL_UNSIGNED_BYTE,
	 buffer);

	free(buffer);
}

void mult_by_shadow_matrix(GLfloat plane[4], GLfloat light[4]) {
	GLfloat m[4][4];
	GLfloat dot = plane[0]*light[0] + plane[1]*light[1] + plane[2]*light[2] +
	 plane[3]*light[3];

	m[0][0] = dot - light[0]*plane[0];
	m[1][0] =     - light[0]*plane[1];
	m[2][0] =     - light[0]*plane[2];
	m[3][0] =     - light[0]*plane[3];

	m[0][1] =     - light[1]*plane[0];
	m[1][1] = dot - light[1]*plane[1];
	m[2][1] =     - light[1]*plane[2];
	m[3][1] =     - light[1]*plane[3];

	m[0][2] =     - light[2]*plane[0];
	m[1][2] =     - light[2]*plane[1];
	m[2][2] = dot - light[2]*plane[2];
	m[3][2] =     - light[2]*plane[3];

	m[0][3] =     - light[3]*plane[0];
	m[1][3] =     - light[3]*plane[1];
	m[2][3] =     - light[3]*plane[2];
	m[3][3] = dot - light[3]*plane[3];

	glMultMatrixf(&m[0][0]);
}

void draw_model(int mode) {
	if(mode == MODE_SHADOW) {
		glColor4f(0.0,0.0,0.0, 0.8);
	} else {
		if(appdata.lighting) {
			glEnable(GL_LIGHTING);
		}
		if(appdata.texturing) {
			glEnable(GL_TEXTURE_2D);
			glBindTexture(GL_TEXTURE_2D, appdata.tex_model);
		}
		if(!appdata.color_texture) {
			glColor3f(1.0, 1.0, 1.0);
		}
	}

	glBegin(GL_QUADS);
	 /* Top: */
	if(mode == MODE_NORMAL) {
		if(appdata.color_texture) {
			glColor3f(1.0, 0.0, 0.0);
		}
		glNormal3f(0,1,0);
	}

	glTexCoord2i(0,0);
	glVertex3i(0,1,0);
	glTexCoord2i(0,2);
	glVertex3i(0,1,2);
	glTexCoord2i(1,2);
	glVertex3i(1,1,2);
	glTexCoord2i(1,0);
	glVertex3i(1,1,0);

	glTexCoord2i(0,0);
	glVertex3i(1,1,0);
	glTexCoord2i(0,1);
	glVertex3i(1,1,1);
	glTexCoord2i(2,1);
	glVertex3i(3,1,1);
	glTexCoord2i(2,0);
	glVertex3i(3,1,0);

	 /* Front: */
	if(mode == MODE_NORMAL) {
		if(appdata.color_texture) {
			glColor3f(1.0, 1.0, 0.0);
		}
		glNormal3f(0,0,1);
	}

	glTexCoord2i(1,0);
	glVertex3i(0,0,2);
	glTexCoord2i(0,0);
	glVertex3i(1,0,2);
	glTexCoord2i(0,1);
	glVertex3i(1,1,2);
	glTexCoord2i(1,1);
	glVertex3i(0,1,2);

	glTexCoord2i(2,0);
	glVertex3i(1,0,1);
	glTexCoord2i(0,0);
	glVertex3i(3,0,1);
	glTexCoord2i(0,1);
	glVertex3i(3,1,1);
	glTexCoord2i(2,1);
	glVertex3i(1,1,1);

	 /* Left: */
	if(mode == MODE_NORMAL) {
		if(appdata.color_texture) {
			glColor3f(0.0, 1.0, 0.0);
		}
		glNormal3f(-1,0,0);
	}

	glTexCoord2i(0,0);
	glVertex3i(0,0,0);
	glTexCoord2i(0,2);
	glVertex3i(0,0,2);
	glTexCoord2i(1,2);
	glVertex3i(0,1,2);
	glTexCoord2i(1,0);
	glVertex3i(0,1,0);

	 /* Right: */
	if(mode == MODE_NORMAL) {
		if(appdata.color_texture) {
			glColor3f(0.0, 1.0, 1.0);
		}
		glNormal3f(1,0,0);
	}

	glTexCoord2i(1,1);
	glVertex3i(1,0,2);
	glTexCoord2i(1,0);
	glVertex3i(1,0,1);
	glTexCoord2i(0,0);
	glVertex3i(1,1,1);
	glTexCoord2i(0,1);
	glVertex3i(1,1,2);

	glTexCoord2i(1,1);
	glVertex3i(3,0,1);
	glTexCoord2i(1,0);
	glVertex3i(3,0,0);
	glTexCoord2i(0,0);
	glVertex3i(3,1,0);
	glTexCoord2i(0,1);
	glVertex3i(3,1,1);

	 /* Back: */
	if(mode == MODE_NORMAL) {
		if(appdata.color_texture) {
			glColor3f(0.0, 0.0, 1.0);
		}
		glNormal3f(0,0,-1);
	}

	glTexCoord2i(3,0);
	glVertex3i(3,0,0);
	glTexCoord2i(0,0);
	glVertex3i(0,0,0);
	glTexCoord2i(0,1);
	glVertex3i(0,1,0);
	glTexCoord2i(3,1);
	glVertex3i(3,1,0);

	 /* Bottom: */
	if(mode == MODE_NORMAL) {
		if(appdata.color_texture) {
			glColor3f(1.0, 0.0, 1.0);
		}
		glNormal3f(0,-1,0);
	}

	glTexCoord2i(0,0);
	glVertex3i(1,0,0);
	glTexCoord2i(0,2);
	glVertex3i(1,0,2);
	glTexCoord2i(1,2);
	glVertex3i(0,0,2);
	glTexCoord2i(1,0);
	glVertex3i(0,0,0);

	glTexCoord2i(0,0);
	glVertex3i(3,0,0);
	glTexCoord2i(0,1);
	glVertex3i(3,0,1);
	glTexCoord2i(2,1);
	glVertex3i(1,0,1);
	glTexCoord2i(2,0);
	glVertex3i(1,0,0);
	glEnd();

	if(mode == MODE_NORMAL) {
		if(appdata.lighting) {
			glDisable(GL_LIGHTING);
		}
		if(appdata.texturing) {
			glDisable(GL_TEXTURE_2D);
		}
	}
}
void draw_plane(int mode) {
	if(mode != MODE_SHADOW) {
		if(appdata.texturing) {
			glEnable(GL_TEXTURE_2D);
			glBindTexture(GL_TEXTURE_2D, appdata.tex_ground);
		}
		if(appdata.lighting) {
			glEnable(GL_LIGHTING);
		}
	}
	if(mode == MODE_REFLECT) {
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glColor4f(1.0,1.0,1.0, 0.5);
	} else {
		glColor3f(1.0,1.0,1.0);
	}
	glBegin(GL_QUADS);
	glNormal3f(0,1,0);
	glTexCoord2d(0,8);
	glVertex3f(-4.0, 0.0,  4.0);
	glTexCoord2d(8,8);
	glVertex3f( 4.0, 0.0,  4.0);
	glTexCoord2d(8,0);
	glVertex3f( 4.0, 0.0, -4.0);
	glTexCoord2d(0,0);
	glVertex3f(-4.0, 0.0, -4.0);
	if(mode != MODE_SHADOW) {
		glColor4f(1.0,1.0,1.0, 1.0);
		 /* Front: */
		glNormal3f(0,0,1);
		glTexCoord2d(0,0);
		glVertex3f(-4.0,  0.0,  4.0);
		glTexCoord2d(0,1);
		glVertex3f(-4.0, -1.0,  4.0);
		glTexCoord2d(8,1);
		glVertex3f( 4.0, -1.0,  4.0);
		glTexCoord2d(8,0);
		glVertex3f( 4.0,  0.0,  4.0);
		 /* Left: */
		glNormal3f(-1,0,0);
		glTexCoord2d(0,0);
		glVertex3f(-4.0,  0.0, -4.0);
		glTexCoord2d(0,1);
		glVertex3f(-4.0, -1.0, -4.0);
		glTexCoord2d(8,1);
		glVertex3f(-4.0, -1.0,  4.0);
		glTexCoord2d(8,0);
		glVertex3f(-4.0,  0.0,  4.0);
		 /* Back: */
		glNormal3f(0,0,-1);
		glTexCoord2d(8,0);
		glVertex3f( 4.0,  0.0, -4.0);
		glTexCoord2d(8,1);
		glVertex3f( 4.0, -1.0, -4.0);
		glTexCoord2d(0,1);
		glVertex3f(-4.0, -1.0, -4.0);
		glTexCoord2d(0,0);
		glVertex3f(-4.0,  0.0, -4.0);
		 /* Right: */
		glNormal3f(1,0,0);
		glTexCoord2d(8,0);
		glVertex3f( 4.0,  0.0,  4.0);
		glTexCoord2d(8,1);
		glVertex3f( 4.0, -1.0,  4.0);
		glTexCoord2d(0,1);
		glVertex3f( 4.0, -1.0, -4.0);
		glTexCoord2d(0,0);
		glVertex3f( 4.0,  0.0, -4.0);
		 /* Bottom: */
		glNormal3f(0,-1,0);
		glTexCoord2d(0,0);
		glVertex3f(-4.0, -1.0, -4.0);
		glTexCoord2d(8,0);
		glVertex3f( 4.0, -1.0, -4.0);
		glTexCoord2d(8,8);
		glVertex3f( 4.0, -1.0,  4.0);
		glTexCoord2d(0,8);
		glVertex3f(-4.0, -1.0,  4.0);
	}
	glEnd();
	if(mode == MODE_REFLECT) {
		glDisable(GL_BLEND);
	}
	if(mode != MODE_SHADOW) {
		if(appdata.texturing) {
			glDisable(GL_TEXTURE_2D);
		}
		if(appdata.lighting) {
			glEnable(GL_LIGHTING);
		}
	}
}

void init_gl(void) {
	load_textures();

	glClearColor(0.0, 0.0, 0.0, 0.0);
	glClearStencil(0);
	glShadeModel(GL_FLAT);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_LIGHT0);
}

void resize_scene(void) {
	int w = appdata.w;
	int h = appdata.h;
	double ratio;

	glViewport(0, 0, w, h);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	if(h == 0) h = 1;
	ratio = (double)w/(double)h;
	gluPerspective(45.0, ratio, 4.0, 20.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	 /* Position camera: */
	glTranslatef(0.0, 0.0, -10.0);
}

void draw_stuff(double t, double angle, double pitch) {
	int i;
	double phi;
	GLfloat light_diffuse[4] = {1.0, 1.0, 1.0, 1.0};
	GLfloat plane_eq[4] = {0.0, 1.0, 0.0, 0.0};

	light_diffuse[0] = fmaxf(0.0, 0.5+appdata.light_pos[1]);
	light_diffuse[1] = fmaxf(0.0, 0.5+appdata.light_pos[1]+0.1*appdata.light_pos[0]);
	light_diffuse[2] = fmaxf(0.0, 0.5+appdata.light_pos[1]+0.3*appdata.light_pos[0]);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glClearColor(light_diffuse[0], light_diffuse[1], light_diffuse[2], 0.0);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

	glPushMatrix();
	glRotatef(pitch, 1.0, 0.0, 0.0);
	glRotatef(angle, 0.0, 1.0, 0.0);
	glTranslatef(0.0, -0.5, 0.0);
	glLightfv(GL_LIGHT0, GL_POSITION, appdata.light_pos);

	if(appdata.shadowing || appdata.reflection) {
		 /* Create stencil: */
		glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
		glDepthMask(GL_FALSE);
		glEnable(GL_STENCIL_TEST);
		glStencilFunc(GL_ALWAYS, 1, 0xFFFFFFFF);
		glStencilOp(GL_REPLACE, GL_REPLACE, GL_REPLACE);
		draw_plane(MODE_SHADOW);
		glStencilFunc(GL_EQUAL, 1, 0xFFFFFFFF);
		glDepthMask(GL_TRUE);
		glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
		glDisable(GL_STENCIL_TEST);
	}

	if(appdata.reflection) {
		 /* Draw reflection: */
		glEnable(GL_STENCIL_TEST);
		glCullFace(GL_FRONT);
//		glClear(GL_DEPTH_BUFFER_BIT);
//		glEnable(GL_DEPTH_TEST);
		glPushMatrix();
		glScalef(1.0, -1.0, 1.0);
		glLightfv(GL_LIGHT0, GL_POSITION, appdata.light_pos);
		glTranslatef(-0.5, 2.0, 0.0);
		glRotatef(360.0*t, 4.0, 2.0, 1.0);
		glTranslatef(-1.0, -0.5, -1.0);
		glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
		draw_model(MODE_NORMAL);
		glPopMatrix();
		glLightfv(GL_LIGHT0, GL_POSITION, appdata.light_pos);
		glCullFace(GL_BACK);
		glDisable(GL_STENCIL_TEST);
	}

	 /* Draw ground: */
	if(appdata.reflection) {
		draw_plane(MODE_REFLECT);
	} else {
		draw_plane(MODE_NORMAL);
	}

	if(appdata.shadowing) {
		 /* Draw shadow: */
		glEnable(GL_STENCIL_TEST);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDisable(GL_DEPTH_TEST);
		glPushMatrix();
		mult_by_shadow_matrix(plane_eq, appdata.light_pos);
		glTranslatef(-0.5, 2.0, 0.0);
		glRotatef(360.0*t, 4.0, 2.0, 1.0);
		glTranslatef(-1.0, -0.5, -1.0);
		glStencilOp(GL_KEEP, GL_KEEP, GL_ZERO);
		draw_model(MODE_SHADOW);
		glPopMatrix();
		glEnable(GL_DEPTH_TEST);
		glDisable(GL_BLEND);
		glDisable(GL_STENCIL_TEST);
	}

	 /* Draw model: */
	glPushMatrix();
	glTranslatef(-0.5, 2.0, 0.0);
	glRotatef(360.0*t, 4.0, 2.0, 1.0);
	glTranslatef(-1.0, -0.5, -1.0);
	draw_model(MODE_NORMAL);
	glPopMatrix();

	glPopMatrix();

	if(appdata.doublebuf) {
		glXSwapBuffers(appdata.disp, appdata.win);
	}
}
