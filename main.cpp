#include <stdio.h>
#include <Windows.h>
#include <GL/gl.h>
#include "SDL2/include/SDL.h"
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <stack>

using namespace std;

stack<int> s;

struct Vector3
{
	float x, y, z;
};

struct Matrix {
	float e[16];
};

struct Camera {
	Vector3 p;
};

struct Quat {
  float x,y,z,w;
};

struct Cube_info {
	Vector3 p;//centre_position
	Vector3 c[3];//color
  Quat orientation;
  Quat target_orientation;
};

#define MATH_PI 3.141592653589793238f
float to_radians(float d)
{
	return MATH_PI / 180.0f * d;
}

float lerp(float from, float to, float alpha) {
	return from + (to - from) * alpha;
}

int get_rand_move()
{
  return rand()%12;
}

Matrix translation(Vector3 t) {
	Matrix m = {
		1.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f, 0.0f,
		t.x,t.y, t.z, 1.0f,
	};
	return m;
}

Matrix rotation_z(float angle) {
	float c = cosf(angle);
	float s = sinf(angle);
	Matrix m = {
		c,       s, 0.0f, 0.0f,
		-s,       c, 0.0f, 0.0f,
		0.0f,  0.0f, 1.0f, 0.0f,
		0.0f,  0.0f, 0.0f, 1.0f,
	};
	return m;
}

Matrix rotation_x(float angle) {
	float c = cosf(angle);
	float s = sinf(angle);
	Matrix m = {
		1.0f,  0.0f, 0.0f, 0.0f,
		0.0f,  c, -s, 0.0f,
		0.0f,  s, c, 0.0f,
		0.0f,  0.0f, 0.0f, 1.0f,
	};
	return m;
}

Matrix rotation_y(float angle) {
	float c = cosf(angle);
	float s = sinf(angle);
	Matrix m = {
		c,  0.0f, s, 0.0f,
		0.0f,  1.0f, 0.0f, 0.0f,
		-s,  0.0f, c, 0.0f,
		0.0f,  0.0f, 0.0f, 1.0f,
	};
	return m;
}

Matrix scalar(Vector3 s) {
	Matrix m = {
		s.x, 0.0f, 0.0f, 0.0f,
		0.0f, s.y, 0.0f, 0.0f,
		0.0f, 0.0f, s.z, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f,
	};
	return m;
}

Matrix perspective_projection(float fov, float aspect_ratio, float n, float f) {
  float  cot = 1.0f / tanf(fov * 0.5f);
  float  fpn = f + n;
  float  fmn = f - n;
  Matrix m = {cot / aspect_ratio,0.0f, 0.0f, 0.0f,
              0.0f, cot, 0.0f, 0.0f,
              0.0f, 0.0f,-fpn / fmn, -1.0f,
              0.0f, 0.0f,-2.0f*f*n/fmn, 0.0f};
  return m;
}

Quat quat_identity() {
  return Quat{0, 0, 0, 1};
}

Quat normalize(Quat q) {
  float len = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  float swrtf(len);
  if (len !=0) {
    q.x /= len;
    q.y /= len;
    q.z /= len;
    q.w /= len;
    return q;
  }
  return Quat{0,0,0,1};
}

Quat lerp(Quat from, Quat to, float alpha) {
  Quat res;
  res.x = from.x + (to.x - from.x) * alpha;
  res.y = from.y + (to.y - from.y) * alpha;
  res.z = from.z + (to.z - from.z) * alpha;
  res.w = from.w + (to.w - from.w) * alpha;
  return normalize(res);
}

Quat quat_mul(Quat q1, Quat q2) {
  float a = q1.w;
  float b = q1.x;
  float c = q1.y;
  float d = q1.z;

  float e = q2.w;
  float f = q2.x;
  float g = q2.y;
  float h = q2.z;

  Quat res;
  res.w = a * e - b * f - c * g - d * h;
  res.x = a * f + b * e + c * h - d * g;
  res.y = a * g - b * h + c * e + d * f;
  res.z = a * h + b * g - c * f + d * e;
  return res;
}

Quat quat_angle_axis(Vector3 axis, float angle) {
  float r = cosf(angle * 0.5f);
  float s = sinf(angle * 0.5f);
  float i = s * axis.x;
  float j = s * axis.y;
  float k = s * axis.z;
  return Quat{i, j, k, r};
}

Matrix quat_get_matrix(Quat q) {
  float i = q.x;
  float j = q.y;
  float k = q.z;
  float r = q.w;

  float ii = i * i;
  float jj = j * j;
  float kk = k * k;

  float ij = i * j;
  float jk = j * k;
  float kr = k * r;
  float jr = j * r;
  float ir = i * r;
  float ik = i * k;

  Matrix m;

  m.e[0] = 1 - 2 * (jj + kk);
  m.e[4] = 2 * (ij - kr);
  m.e[8] = 2 * (ik + jr);
  m.e[12] = 0;

  m.e[1] = 2 * (ij + kr);
  m.e[5] = 1 - 2 * (ii + kk);
  m.e[9] = 2 * (jk - ir);
  m.e[13] = 0;

  m.e[2] = 2 * (ik - jr);
  m.e[6] = 2 * (jk + ir);
  m.e[10] = 1 - 2 * (ii + jj);
  m.e[14] = 0;

  m.e[3] = 0;
  m.e[7] = 0;
  m.e[11] = 0;
  m.e[15] = 1;

  return m;
}

void draw_cube(Vector3 a, Vector3 b, Vector3 c)
{
	//front face
	glNormal3f(0.0f,0.0f,1.0f);
	glColor3f(a.x, a.y, a.z);
	glVertex3f(-0.5f, -0.5f, 0.5f);
	glVertex3f(-0.5f, 0.5f, 0.5f);
	glVertex3f(0.5f, 0.5f, 0.5f);

	glNormal3f(0.0f,0.0f,1.0f);
	glVertex3f(-0.5f, -0.5f, 0.5f);
	glVertex3f(0.5f, 0.5f, 0.5f);
	glVertex3f(0.5f, -0.5f, 0.5f);

	//right face
	glNormal3f(1.0f,0.0f,0.0f);
	glColor3f(b.x, b.y, b.z);
	glVertex3f(0.5f, -0.5f, 0.5f);
	glVertex3f(0.5f, 0.5f, 0.5f);
	glVertex3f(0.5f, 0.5f, -0.5f);

	glNormal3f(1.0f,0.0f,0.0f);
	glVertex3f(0.5f, -0.5f, 0.5f);
	glVertex3f(0.5f, 0.5f, -0.5f);
	glVertex3f(0.5f, -0.5f, -0.5f);

	//back face
	glNormal3f(0.0f,0.0f,-1.0f);
	glColor3f(a.x, a.y, a.z);
	glVertex3f(0.5f, -0.5f, -0.5f);
	glVertex3f(0.5f, 0.5f, -0.5f);
	glVertex3f(-0.5f, 0.5f, -0.5f);

	glNormal3f(0.0f,0.0f,-1.0f);
	glVertex3f(0.5f, -0.5f, -0.5f);
	glVertex3f(-0.5f, 0.5f, -0.5f);
	glVertex3f(-0.5f, -0.5f, -0.5f);

	//leftface
	glNormal3f(-1.0f,0.0f,0.0f);
	glColor3f(b.x, b.y, b.z);
	glVertex3f(-0.5f, -0.5f, -0.5f);
	glVertex3f(-0.5f, 0.5f, -0.5f);
	glVertex3f(-0.5f, 0.5f, 0.5f);

	glNormal3f(-1.0f,0.0f,0.0f);
	glVertex3f(-0.5f, -0.5f, -0.5f);
	glVertex3f(-0.5f, 0.5f, 0.5f);
	glVertex3f(-0.5f, -0.5f, 0.5f);

	//top face
	glNormal3f(0.0f,1.0f,0.0f);
	glColor3f(c.x, c.y, c.z);
	glVertex3f(-0.5f, 0.5f, 0.5f);
	glVertex3f(-0.5f, 0.5f, -0.5f);
	glVertex3f(0.5f, 0.5f, -0.5f);

	glNormal3f(0.0f,1.0f,0.0f);
	glVertex3f(-0.5f, 0.5f, 0.5f);
	glVertex3f(0.5f, 0.5f, -0.5f);
	glVertex3f(0.5f, 0.5f, 0.5f);

	//buttom face
	glNormal3f(0.0f,-1.0f,0.0f);
	glColor3f(c.x, c.y, c.z);
	glVertex3f(-0.5f, -0.5f, -0.5f);
	glVertex3f(-0.5f, -0.5f, 0.5f);
	glVertex3f(0.5f, -0.5f, 0.5f);

	glNormal3f(0.0f,-1.0f,0.0f);
	glVertex3f(-0.5f, -0.5f, -0.5f);
	glVertex3f(0.5f, -0.5f, 0.5f);
	glVertex3f(0.5f, -0.5f, -0.5f);
}

void rotate_fr(Cube_info **positions) {
	Cube_info *temp = positions[0];
	positions[0] = positions[2];
	positions[2] = positions[3];
	positions[3] = positions[1];
	positions[1] = temp;

	float rot = to_radians(90);
	Quat q = quat_angle_axis({0,0,-1},rot);
	positions[0]->target_orientation = quat_mul(q,positions[0]->target_orientation);
	positions[1]->target_orientation = quat_mul(q,positions[1]->target_orientation);
	positions[2]->target_orientation = quat_mul(q,positions[2]->target_orientation);
	positions[3]->target_orientation = quat_mul(q,positions[3]->target_orientation);
}

void rotate_fl(Cube_info **positions) {
	Cube_info *temp = positions[0];
	positions[0] = positions[1];
	positions[1] = positions[3];
	positions[3] = positions[2];
	positions[2] = temp;

	float rot = to_radians(90);
	Quat q = quat_angle_axis({0,0,1},rot);
	positions[0]->target_orientation = quat_mul(q,positions[0]->target_orientation);
	positions[1]->target_orientation = quat_mul(q,positions[1]->target_orientation);
	positions[2]->target_orientation = quat_mul(q,positions[2]->target_orientation);
	positions[3]->target_orientation = quat_mul(q,positions[3]->target_orientation);
}

void rotate_br(Cube_info **positions) {
  Cube_info *temp = positions[7];
  positions[7] = positions[6];
  positions[6] = positions[4];
  positions[4] = positions[5];
  positions[5] = temp;

  float rot = to_radians(90);
  Quat q = quat_angle_axis({0,0,1},rot);
  positions[4]->target_orientation = quat_mul(q,positions[4]->target_orientation);
  positions[5]->target_orientation = quat_mul(q,positions[5]->target_orientation);
  positions[6]->target_orientation = quat_mul(q,positions[6]->target_orientation);
  positions[7]->target_orientation = quat_mul(q,positions[7]->target_orientation);
}

void rotate_bl(Cube_info **positions) {
  Cube_info *temp = positions[7];
  positions[7] = positions[5];
  positions[5] = positions[4];
  positions[4] = positions[6];
  positions[6] = temp;

  float rot = to_radians(90);
  Quat q = quat_angle_axis({0,0,-1},rot);
  positions[4]->target_orientation = quat_mul(q,positions[4]->target_orientation);
  positions[5]->target_orientation = quat_mul(q,positions[5]->target_orientation);
  positions[6]->target_orientation = quat_mul(q,positions[6]->target_orientation);
  positions[7]->target_orientation = quat_mul(q,positions[7]->target_orientation);
}

void rotate_rr(Cube_info **positions) {
	Cube_info *temp = positions[4];
	positions[4] = positions[6];
	positions[6] = positions[2];
	positions[2] = positions[0];
	positions[0] = temp;

	float rot = to_radians(90);
	Quat q = quat_angle_axis({-1,0,0},rot);
	positions[0]->target_orientation = quat_mul(q,positions[0]->target_orientation);
	positions[4]->target_orientation = quat_mul(q,positions[4]->target_orientation);
	positions[6]->target_orientation = quat_mul(q,positions[6]->target_orientation);
	positions[2]->target_orientation = quat_mul(q,positions[2]->target_orientation);
}

void rotate_rl(Cube_info **positions) {
  Cube_info *temp = positions[4];
  positions[4] = positions[0];
  positions[0] = positions[2];
  positions[2] = positions[6];
  positions[6] = temp;

  float rot = to_radians(90);
  Quat q = quat_angle_axis({1,0,0},rot);
  positions[0]->target_orientation = quat_mul(q,positions[0]->target_orientation);
  positions[4]->target_orientation = quat_mul(q,positions[4]->target_orientation);
  positions[6]->target_orientation = quat_mul(q,positions[6]->target_orientation);
  positions[2]->target_orientation = quat_mul(q,positions[2]->target_orientation);
}
void rotate_lr(Cube_info **positions) {
  Cube_info *temp = positions[1];
  positions[1] = positions[3];
  positions[3] = positions[7];
  positions[7] = positions[5];
  positions[5] = temp;

  float rot = to_radians(90);
  Quat q = quat_angle_axis({1,0,0},rot);
  positions[1]->target_orientation = quat_mul(q,positions[1]->target_orientation);
  positions[3]->target_orientation = quat_mul(q,positions[3]->target_orientation);
  positions[5]->target_orientation = quat_mul(q,positions[5]->target_orientation);
  positions[7]->target_orientation = quat_mul(q,positions[7]->target_orientation);
}

void rotate_ll(Cube_info **positions) {
  Cube_info *temp = positions[1];
  positions[1] = positions[5];
  positions[5] = positions[7];
  positions[7] = positions[3];
  positions[3] = temp;

  float rot = to_radians(90);
  Quat q = quat_angle_axis({-1,0,0},rot);
  positions[1]->target_orientation = quat_mul(q,positions[1]->target_orientation);
  positions[3]->target_orientation = quat_mul(q,positions[3]->target_orientation);
  positions[5]->target_orientation = quat_mul(q,positions[5]->target_orientation);
  positions[7]->target_orientation = quat_mul(q,positions[7]->target_orientation);
}

void rotate_ur(Cube_info **positions) {
  Cube_info *temp = positions[2];
  positions[2] = positions[6];
  positions[6] = positions[7];
  positions[7] = positions[3];
  positions[3] = temp;

  float rot = to_radians(90);
  Quat q = quat_angle_axis({0,-1,0},rot);
  positions[2]->target_orientation = quat_mul(q,positions[2]->target_orientation);
  positions[3]->target_orientation = quat_mul(q,positions[3]->target_orientation);
  positions[6]->target_orientation = quat_mul(q,positions[6]->target_orientation);
  positions[7]->target_orientation = quat_mul(q,positions[7]->target_orientation);
}

void rotate_ul(Cube_info **positions) {
  Cube_info *temp = positions[2];
  positions[2] = positions[3];
  positions[3] = positions[7];
  positions[7] = positions[6];
  positions[6] = temp;

  float rot = to_radians(90);
  Quat q = quat_angle_axis({0,1,0},rot);
  positions[2]->target_orientation = quat_mul(q,positions[2]->target_orientation);
  positions[3]->target_orientation = quat_mul(q,positions[3]->target_orientation);
  positions[6]->target_orientation = quat_mul(q,positions[6]->target_orientation);
  positions[7]->target_orientation = quat_mul(q,positions[7]->target_orientation);
}

void rotate_dr(Cube_info **positions) {
  Cube_info *temp = positions[0];
  positions[0] = positions[1];
  positions[1] = positions[5];
  positions[5] = positions[4];
  positions[4] = temp;

  float rot = to_radians(90);
  Quat q = quat_angle_axis({0,1,0},rot);
  positions[0]->target_orientation = quat_mul(q,positions[0]->target_orientation);
  positions[1]->target_orientation = quat_mul(q,positions[1]->target_orientation);
  positions[4]->target_orientation = quat_mul(q,positions[4]->target_orientation);
  positions[5]->target_orientation = quat_mul(q,positions[5]->target_orientation);
}

void rotate_dl(Cube_info **positions) {
  Cube_info *temp = positions[0];
  positions[0] = positions[4];
  positions[4] = positions[5];
  positions[5] = positions[1];
  positions[1] = temp;

  float rot = to_radians(90);
  Quat q = quat_angle_axis({0,-1,0},rot);
  positions[0]->target_orientation = quat_mul(q,positions[0]->target_orientation);
  positions[1]->target_orientation = quat_mul(q,positions[1]->target_orientation);
  positions[4]->target_orientation = quat_mul(q,positions[4]->target_orientation);
  positions[5]->target_orientation = quat_mul(q,positions[5]->target_orientation);
}



int main(int argc, char *argv[])
{
	SDL_Window *window;
	SDL_Renderer *renderer;

	int w = 600, h = 600;
	
	srand(time(0));

	SDL_Init(SDL_INIT_EVERYTHING);
	window = SDL_CreateWindow("An SDL2 window", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, w, h, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);

	if (window == NULL)
	{
		printf("Could not create window: %s\n", SDL_GetError());
		return 1;
	}

	Uint32 windowID = SDL_GetWindowID(window);
	SDL_GLContext glcontext = SDL_GL_CreateContext(window);

	//VISUAL SURFACE DETECTION / DEPTH BUFFER
	glEnable(GL_DEPTH_TEST);

	//ILLUMINATION AND SURFACE RENDERING
	//GOURAUD SHADING
	glShadeModel(GL_SMOOTH);

	//LIGHTING
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_COLOR_MATERIAL);
	float pos[] = {-2.0f,2.0f,0.7f,1};
	glLightfv(GL_LIGHT0,GL_POSITION, pos);
	float dif[] = {1.0f,1.0f,1.0f};
	glLightfv(GL_LIGHT0,GL_DIFFUSE,dif);
	float amb[] = {0.1f,0.0f,0.1f};
	glLightfv(GL_LIGHT0,GL_AMBIENT,amb);

	bool solving = false;

	Camera camera;
	camera.p = { 0,0,10 };

	const Vector3 RED = { 0.8,0.1,0.1 };
	const Vector3 BLUE = { 0.1,0.1,0.8 };
	const Vector3 YELLOW = { 0.8,0.8,0.1 };
	const Vector3 GREEN = { 0.1,0.8,0.1 };
	const Vector3 WHITE = { 0.8,0.8,0.8 };
	const Vector3 PURPLE = { 0.1,0.9,0.9 };
	
	Cube_info cubes[8];
	cubes[0].p = { 0.5, -0.5,0.5 };
	cubes[0].c[0] = RED;
	cubes[0].c[1] = GREEN;
	cubes[0].c[2] = BLUE;
	cubes[0].orientation = quat_identity();
	cubes[0].target_orientation = quat_identity();

	cubes[1].p = { -0.5,-0.5,0.5 };
	cubes[1].c[0] = RED;
	cubes[1].c[1] = PURPLE;
	cubes[1].c[2] = BLUE;
	cubes[1].orientation = quat_identity();
	cubes[1].target_orientation = quat_identity();

	cubes[2].p = { 0.5,0.5,0.5 };
	cubes[2].c[0] = RED;
	cubes[2].c[1] = GREEN;
	cubes[2].c[2] = YELLOW;
	cubes[2].orientation = quat_identity();
	cubes[2].target_orientation = quat_identity();

	cubes[3].p = { -0.5,0.5,0.5 };
	cubes[3].c[0] = RED;
	cubes[3].c[1] = PURPLE;
	cubes[3].c[2] = YELLOW;
	cubes[3].orientation = quat_identity();
	cubes[3].target_orientation = quat_identity();

	cubes[4].p = { 0.5, -0.5,-0.5 };
	cubes[4].c[0] = WHITE;
	cubes[4].c[1] = GREEN;
	cubes[4].c[2] = BLUE;
	cubes[4].orientation = quat_identity();
	cubes[4].target_orientation = quat_identity();

	cubes[5].p = { -0.5,-0.5,-0.5 };
	cubes[5].c[0] = WHITE;
	cubes[5].c[1] = PURPLE;
	cubes[5].c[2] = BLUE;
	cubes[5].orientation = quat_identity();
	cubes[5].target_orientation = quat_identity();

	cubes[6].p = { 0.5,0.5,-0.5 };
	cubes[6].c[0] = WHITE;
	cubes[6].c[1] = GREEN;
	cubes[6].c[2] = YELLOW;
	cubes[6].orientation = quat_identity();
	cubes[6].target_orientation = quat_identity();

	cubes[7].p = { -0.5,0.5,-0.5 };
	cubes[7].c[0] = WHITE;
	cubes[7].c[1] = PURPLE;
	cubes[7].c[2] = YELLOW;
	cubes[7].orientation = quat_identity();
	cubes[7].target_orientation = quat_identity();

	Cube_info *positions[8];
	positions[0] = cubes+0;
	positions[1] = cubes+1;
	positions[2] = cubes+2;
	positions[3] = cubes+3;
	positions[4] = cubes+4;
	positions[5] = cubes+5;
	positions[6] = cubes+6;
	positions[7] = cubes+7;

	int choice;
	
	bool making_a_move = false;

	float angle_x,angle_y,angle_z;
	angle_z = angle_y = angle_x = 0.0f;

	int rotate_choice = 0;

	Matrix rotate_x,rotate_y,rotate_z;
	rotate_x = rotate_y = rotate_z = rotation_y(to_radians(0.0f));
	
	bool running = true;
	while (running)
	{
		float forward_key = 0, right_key=0;
		
		choice = 12;

		SDL_Event event;
		while (SDL_PollEvent(&event))
		{
			if (event.type == SDL_QUIT)
			{
				running = false;
				break;
			}

			switch (event.type)
			{
				case SDL_WINDOWEVENT:
				{
					if (event.window.windowID == windowID)
					{
						switch (event.window.event)
						{

							case SDL_WINDOWEVENT_SIZE_CHANGED:
							{
								w = event.window.data1;
								h = event.window.data2;
								break;
							}
						}
					}
				}
				break;

				case SDL_KEYUP:
				{
					if (!solving) 
					{
						bool ctrl = (KMOD_CTRL & SDL_GetModState());
						switch(event.key.keysym.sym) 
						{
						case SDLK_ESCAPE:
							running = 0;
							break;
						
						case SDLK_5:
						case SDLK_KP_5:
						{
							if(ctrl==0)
							{
								rotate_fr((Cube_info **)&positions);
								s.push(0);
							}
							else
							{
								rotate_fl((Cube_info **)&positions);
								s.push(1);
							}
							break;
						}
						
						case SDLK_0:
						case SDLK_KP_0:
						{
							if(ctrl==0)
							{
								rotate_br((Cube_info **)&positions);
								s.push(6);
							}
							else
							{
								rotate_bl((Cube_info **)&positions);
								s.push(7);
							}
							break;
						}
						
						case SDLK_6:
						case SDLK_KP_6:
						{
							if(ctrl==0)
							{
								rotate_rr((Cube_info **)&positions);
								s.push(4);
							}
							else
							{
								rotate_rl((Cube_info **)&positions);
								s.push(5);
							}
							break;
						}
						
						case SDLK_4:
						case SDLK_KP_4:
						{
							if(ctrl==0)
							{
								rotate_lr((Cube_info **)&positions);
								s.push(2);
							}
							else
							{
								rotate_ll((Cube_info **)&positions);
								s.push(3);
							}
							break;
						}
						
						case SDLK_8:
						case SDLK_KP_8:
						{
							if(ctrl==0)
							{
								rotate_ur((Cube_info **)&positions);
								s.push(8);
							}
							else
							{
								rotate_ul((Cube_info **)&positions);
								s.push(9);
							}
							break;
						}
						
						case SDLK_2:
						case SDLK_KP_2:
						{
							if(ctrl==0)
							{
								rotate_dr((Cube_info **)&positions);
								s.push(10);
							}
							else
							{
								rotate_dl((Cube_info **)&positions);
								s.push(11);
							}
							break;
						}

						case SDLK_r:
						{
							solving = true;
							break;
						}		
					}			
				}			
				}
				break;

				case SDL_KEYDOWN:
				{
					switch (event.key.keysym.sym)
					{
						case SDLK_UP:
							forward_key = -1;
							break;
						case SDLK_DOWN:
							forward_key = 1;
							break;
						case SDLK_LEFT:
							right_key = -1;
							angle_y -= 10.0f;
							rotate_choice = 3;
							break;
						case SDLK_RIGHT:
							right_key = 1;
							angle_y += 10.0f;
							rotate_choice = 3;
							break;
						case SDLK_w:
							angle_x += 10.0f;
							rotate_choice = 1;
							break;
						case SDLK_s:
							angle_x -= 10.0f;
							rotate_choice = 1;
							break;
						case SDLK_d:
							angle_z += 10.0f;
							rotate_choice = 2;
							break;
						case SDLK_a:
							angle_z -= 10.0f;
							rotate_choice = 2;
							break;
							
						case SDLK_SPACE:
							if (!making_a_move)
								choice = get_rand_move();
							s.push(choice);
							break;
					}
				}
			}
		}

    if (solving) 
    {
      int i=0;
      int moves_doing = 0;

      if (!s.empty()) 
      {
        if(!making_a_move)
        {
          switch(s.top())
          {
            case 1: rotate_fr(positions); break;
            case 0: rotate_fl(positions); break;
            case 3: rotate_lr(positions); break;
            case 2: rotate_ll(positions); break;
            case 5: rotate_rr(positions); break;
            case 4: rotate_rl(positions); break;
            case 7: rotate_br(positions); break;
            case 6: rotate_bl(positions); break;
            case 9: rotate_ur(positions); break;
            case 8: rotate_ul(positions); break;
            case 11: rotate_dr(positions); break;
            case 10: rotate_dl(positions); break;
          }
          s.pop();
        }

          for (auto &c : cubes) {
          c.orientation = lerp(c.orientation, c.target_orientation, 0.005);
          if (fabsf(c.orientation.x - c.target_orientation.x) < 0.000001f &&
            fabsf(c.orientation.y - c.target_orientation.y) < 0.000001f &&
            fabsf(c.orientation.z - c.target_orientation.z) < 0.000001f &&
            fabsf(c.orientation.w - c.target_orientation.w) < 0.000001f) {
              
            } else {
              moves_doing |= (1 << i);
            }
          i += 1;
        }
        
        #if 1
        if (moves_doing == 0)
          making_a_move = false;
        else
          making_a_move = true;
        #endif
    
      } 
      else 
      {
        solving = false;
      }
    } 
    else 
    {
      switch(choice)
      {
        case 0: rotate_fr((Cube_info **)&positions); break;
        case 1: rotate_fl((Cube_info **)&positions); break;
        case 2: rotate_lr((Cube_info **)&positions); break;
        case 3: rotate_ll((Cube_info **)&positions); break;
        case 4: rotate_rr((Cube_info **)&positions); break;
        case 5: rotate_rl((Cube_info **)&positions); break;
        case 6: rotate_br((Cube_info **)&positions); break;
        case 7: rotate_bl((Cube_info **)&positions); break;
        case 8: rotate_ur((Cube_info **)&positions); break;
        case 9: rotate_ul((Cube_info **)&positions); break;
        case 10: rotate_dr((Cube_info **)&positions); break;
        case 11: rotate_dl((Cube_info **)&positions); break;
      }
    }
		
		float aspect_ratio = (float)w / (float)h;

		camera.p.z += forward_key*0.1;

		int i = 0;
		int moves_doing = 0;
		for (auto &c : cubes) {
			c.orientation = lerp(c.orientation, c.target_orientation, 0.01);
			if (fabsf(c.orientation.x - c.target_orientation.x) < 0.1f &&
				fabsf(c.orientation.y - c.target_orientation.y) < 0.1f &&
				fabsf(c.orientation.z - c.target_orientation.z) < 0.1f &&
				fabsf(c.orientation.w - c.target_orientation.w) < 0.1f) {
					
				} else {
					moves_doing |= (1 << i);
				}
			i += 1;
		}
		
		#if 1
		if (moves_doing == 0)
			making_a_move = false;
		else
			making_a_move = true;
		#endif

		glClearColor(0.2f, 0.2f, 0.2f, 1);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		Matrix view = translation(Vector3 { -camera.p.x,-camera.p.y, -camera.p.z });
		Matrix scale = scalar(Vector3{ 0.95f, 0.95f, 0.95f });
		Matrix proj = perspective_projection(to_radians(30),aspect_ratio,0.1,100);

    switch(rotate_choice)
    {
      case 1:
      {
        rotate_x = rotation_x(to_radians(angle_x));
        break;
      }
      case 2:
      {
        rotate_z = rotation_z(to_radians(angle_z));
        break;
      }
      case 3:
      {
        rotate_y = rotation_y(to_radians(angle_y));
        break;
      }
    }

		glViewport(0, 0, w, h);

		glPushMatrix();
		glMultMatrixf(proj.e);
		
		glPushMatrix();
		glMultMatrixf(view.e);

		glPushMatrix();
		glMultMatrixf(rotate_x.e);

		glPushMatrix();
		glMultMatrixf(rotate_y.e);

		glPushMatrix();
		glMultMatrixf(rotate_z.e);

		Matrix t, q;
		
		for (auto &c : cubes) {
			q = quat_get_matrix(c.orientation);

			t = translation(c.p);

			glPushMatrix();
			glMultMatrixf(q.e);
			glPushMatrix();
			glMultMatrixf(t.e);
			
			glPushMatrix();
			glMultMatrixf(scale.e);

			glBegin(GL_TRIANGLES);
			draw_cube(c.c[0],c.c[1],c.c[2]);
			glEnd();


			glPopMatrix();
			glPopMatrix();
			glPopMatrix();
		}


		glPopMatrix();
		glPopMatrix();
		glPopMatrix();

		glPopMatrix();
		glPopMatrix();

		SDL_GL_SwapWindow(window);
	}

	SDL_GL_DeleteContext(glcontext);

	SDL_Quit();
	return 0;
}
