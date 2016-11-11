#ifdef __APPLE__
#  pragma clang diagnostic ignored "-Wdeprecated-declarations"
#  include <GLUT/glut.h>
#else
#  include <GL/glut.h>
#endif

#include <iostream>
#include <vector>
#include <cmath>
#include <limits>

#include "ImageIO/ImageIO.h"

#define DIM 3
#define maximum(x, y, z) ((x) > (y)? ((x) > (z)? (x) : (z)) : ((y) > (z)? (y) : (z))) 

/** Special vars */
static const double PI = 3.1415926536;
static const double EPSILON = 1.0e-4;
static const unsigned char ESC = 27;

/** ImageIO handlers */

static int inW, inH, outW, outH;
static int warpFuncNum;
static unsigned char *outPixmap;
static std::string input, output;

static ImageIO ioOrigin = ImageIO();
static ImageIO ioWarped = ImageIO();

/** Warp functions */

float U(float x, float y)
{
  float r = sqrt(x * x + y * y);
  switch(warpFuncNum) {
    case 1:
      // inverse in x direction is sqrt
      return sqrt(x); break;
    case 2:
      return x * cos(r) + y * sin(r); break;
    default:
      return 0.0; break;
  }
}

float V(float x, float y)
{
  float r = sqrt(x * x + y * y);
  switch(warpFuncNum) {
    case 1:
      // inverse in y direction is offset sine
      return 0.5 * (1 + sin(y * PI)); break;
    case 2:
      return -1 * x * sin(r) + y * cos(r); break;
    default:
      return 0.0; break;
  }
}

/*
  Routine to inverse map (x, y) output image spatial coordinates
  into (u, v) input image spatial coordinates

  Call routine with (x, y) spatial coordinates in the output
  image. Returns (u, v) spatial coordinates in the input image,
  after applying the inverse map. 
  Note: (u, v) and (x, y) are not rounded to integers, since they are true spatial coordinates.
 
  inwidth and inheight are the input image dimensions
  outwidth and outheight are the output image dimensions
*/
void inv_map(float x, float y, float &u, float &v,
  int inwidth, int inheight, int outwidth, int outheight) {
  // normalize (x, y) to (0...1, 0...1)
  x /= outwidth;  
  y /= outheight;

  u = U(x, y);                  
  v = V(x, y); 

  // scale normalized (u, v) to pixel coords
  u *= inwidth;     
  v *= inheight;

  // eliminate strange line in center y
  u = (u == inwidth)  ? (u - EPSILON) : u;
  v = (v == inheight) ? (v - EPSILON) : v;
}

/** To draw on screen, outpixmap ALWAYS uses RGBA */
void setupOutPixmap(int w, int h) 
{
  outPixmap = new unsigned char[RGBA * w * h];
  for (int i = 0; i < h; ++i) 
    for (int j = 0; j < w; ++j) 
      for (int channel = 0; channel < RGBA; ++channel)
        /** Init alpha channel to 0, make pixel no color value to be transparent */
        outPixmap[(i * w + j) * RGBA + channel] = 0;
}

// (x, y) is the pixel in output pixmap, inIndexs are the corresponding samples' index in input pixmap.
void requestSamples(int x, int y, int inIndexs[])
{
  float xs[] = {x - 0.5, x, x + 0.5};
  float ys[] = {y - 0.5, y, y + 0.5};
  float us[DIM] = {0}, vs[DIM] = {0};

  int k, l;
  for (int i = 0; i < DIM; ++i) { 
    for (int j = 0; j < DIM; ++j) { 
      inv_map(xs[i], ys[j], us[i], vs[j], inW, inH, outW, outH);
      k = (int)std::floor(vs[j]);
      k = fmax(fmin(k, inH), 0);
      l = (int)std::floor(us[i]);
      l = fmax(fmin(l, inW), 0);
      inIndexs[i * DIM + j] = k * inW + l;
    }
  }
}

bool needSupersampling(int x, int y, float threshold)
{
  float min = std::numeric_limits<float>::max();
  float max = std::numeric_limits<float>::min();
  
  int indexs[DIM * DIM];  
  requestSamples(x, y, indexs);

  float lavg = 0; // luminance average
  unsigned char ls[DIM * DIM];
  for (int i = 0; i < DIM * DIM; ++i) {
    ls[i] = maximum(ioOrigin.pixmap[i + R],ioOrigin.pixmap[i + G], ioOrigin.pixmap[i + B]);
    lavg += ls[i];
  }
  lavg /= DIM * DIM;

  for (int i = 0; i < DIM * DIM; ++i) {
    if (fabsf(ls[i] - lavg) / lavg > threshold)
      return true;
  }
  return false;
}

// calculate average color value at pixel(x, y)
void calculateAverageValue(int x, int y, unsigned char pixel[RGBA])
{
  int indexs[DIM * DIM];
  requestSamples(x, y, indexs);

  int avgR = 0, avgG = 0, avgB = 0, avgA = 0;
  for (int i = 0; i < DIM * DIM; ++i) {
    avgR += ioOrigin.pixmap[i + R];
    avgG += ioOrigin.pixmap[i + G];
    avgB += ioOrigin.pixmap[i + B];
    avgA += ioOrigin.pixmap[i + A];
  }

  avgR /= DIM * DIM; pixel[R] = avgR;
  avgG /= DIM * DIM; pixel[G] = avgG;
  avgB /= DIM * DIM; pixel[B] = avgB;
  avgA /= DIM * DIM; pixel[A] = avgA;
}

void adaptiveSupersampling()
{
  unsigned char pixel[RGBA];
  for (int i = 0; i < outH; ++i) 
    for (int j = 0; j < outW; ++j) 
      if (needSupersampling(j, i, 0.5)) {
        calculateAverageValue(j, i, pixel);
        outPixmap[(i * outW + j) * RGBA + R] = pixel[R];
        outPixmap[(i * outW + j) * RGBA + G] = pixel[G];
        outPixmap[(i * outW + j) * RGBA + B] = pixel[B];
        outPixmap[(i * outW + j) * RGBA + A] = pixel[A];
      }
}

void warp()
{
  int k, l;
  float u, v;

  adaptiveSupersampling();

  for (int i = 0; i < outH; ++i) {
    for (int j = 0; j < outW; ++j) {
      inv_map((float)(j + 0.5), (float)(i + 0.5), u, v, inW, inH, outW, outH);
      k = (int)std::floor(v);
      l = (int)std::floor(u);

      if (k < 0 || k > inH || l < 0 || l > inW) 
        continue;

      for (int channel = 0; channel < RGBA; ++channel) 
        outPixmap[(i * outW + j) * RGBA + channel] = ioOrigin.pixmap[(k * inW + l) * RGBA + channel];
    }
  }
}

bool parseCommandLine(int argc, char* argv[]) 
{
  switch (argc) {
  case 3: case 4:
    if (argv[1][0] != '-') {
      std::cerr << "Usage: warp -a | -b (choose warp function) inimage.png [outimage.png]" << std:: endl;
      return false; break;
    }
    if (argv[1][1] == 'a') {
      warpFuncNum = 1;
    } else if (argv[1][1] == 'b') {
      warpFuncNum = 2;
    }
    input  = argv[2];
    output = argv[3] != NULL ? argv[3] : "output.png";
    return true; break;

  default:
    std::cerr << "Usage: warp -a | -b (choose warp function) inimage.png [outimage.png]" << std:: endl;
    exit(1);
    return false; break;
  }
}

void loadImage()
{
  ioOrigin.loadFile(input);

  inW = ioOrigin.getWidth();
  inH = ioOrigin.getHeight();

  // make output size same to input
  outW = inW;
  outH = inH;
  setupOutPixmap(outW, outH);

  warp();
  ioWarped.setPixmap(outW, outH, outPixmap);
}

void displayOriginWindow() 
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  ioOrigin.draw();
}

void displayWarpedWindow()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  ioWarped.draw();
}

void handleReshape(int width, int height) 
{
  glViewport(0, 0, width, height);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluOrtho2D(0, width, 0, height);
  glMatrixMode(GL_MODELVIEW); 
}

void handleKeyboard(unsigned char key, int x, int y) 
{
  switch(key) {
    case 'w': case 'W': 
      ioWarped.saveImage(output); break;
    case 'q': case 'Q': case ESC: 
      exit(0); break;
  } 
}

int main(int argc, char *argv[])
{
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_SINGLE | GLUT_RGBA);

  if (parseCommandLine(argc, argv)) {
    loadImage();  
  }

  // Origin image window
  glutInitWindowSize(inW, inH);
  glutCreateWindow("Original Image");
  glutDisplayFunc(displayOriginWindow);
  glutKeyboardFunc(handleKeyboard);
  glutReshapeFunc(handleReshape);

  // Warped image window
  glutInitWindowSize(outW, outH);
  glutInitWindowPosition(glutGet(GLUT_WINDOW_X) + inW, 0);
  glutCreateWindow("Warped Image");
  glutDisplayFunc(displayWarpedWindow);
  glutKeyboardFunc(handleKeyboard);
  glutReshapeFunc(handleReshape);

  glutMainLoop();
  
  return 0;
}

