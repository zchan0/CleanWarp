#ifdef __APPLE__
#  pragma clang diagnostic ignored "-Wdeprecated-declarations"
#  include <GLUT/glut.h>
#else
#  include <GL/glut.h>
#endif

#include <iostream>
#include <vector>
#include <cmath>

#include "ImageIO/ImageIO.h"

/** Special vars */

static const double PI = 3.1415926536;
static const unsigned char ESC = 27;

/** ImageIO handlers */

static int inW, inH, outW, outH;
static int warpFuncNum;
static unsigned char *outPixmap;
static std::string input, output;

static ImageIO ioOrigin = ImageIO();
static ImageIO ioWarped = ImageIO();

/** Warp functions */

float X(float u, float v)
{
  return u;
}

float Y(float u, float v)
{
  return v;
}

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
  int inwidth, int inheight, int outwidth, int outheight){
  
  // normalize (x, y) to (0...1, 0...1)
  x /= outwidth;  
  y /= outheight;

  u = U(x, y);                  
  v = V(x, y); 

  // scale normalized (u, v) to pixel coords
  u *= inwidth;     
  v *= inheight;
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

void setOutputSize(int &w, int &h)
{
  int maxX, maxY = 0;
  int *xs = new int[2]; // x coordinates
  int *ys = new int[2]; // y coordinates

  xs[0] = 0;  xs[1] = inW; 
  ys[0] = 0;  ys[1] = inH;

  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      maxX = fmax(maxX, X(xs[i], ys[j]));
      maxY = fmax(maxY, Y(xs[i], ys[j]));
    }
  }

  w = maxX;
  h = maxY;
  // use w and h to setup output pixmap 
  setupOutPixmap(w, h);
}

void warp(int inW, int inH, unsigned char *inPixmap)
{
  int k, l;
  float u, v;

  setOutputSize(outW, outH);

  for (int i = 0; i < outH; ++i) {
    for (int j = 0; j < outW; ++j) {
      inv_map(j, i, u, v, inW, inH, outW, outH);
      k = (int)std::floor(v);
      l = (int)std::floor(u);

      if (k < 0 || k > inH || l < 0 || l > inW) 
        continue;

      for (int channel = 0; channel < RGBA; ++channel) 
        outPixmap[(i * outW + j) * RGBA + channel] = inPixmap[(k * inW + l) * RGBA + channel];
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

  warp(inW, inH, ioOrigin.pixmap);
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

