#include <iostream>
#include <cmath>

#include "Image.h"

Image::Image()
{
	pixmap = NULL;
	floatPixmap = NULL;
	filename = channelname = "";
	width  = height = nchannels = 0;
}

Image::~Image()
{
	// if (pixmap != NULL) delete [] pixmap;
	// if (floatPixmap != NULL) delete [] floatPixmap;
}

Image::Image(const Image& img)
{
	copy(img);
}

Image::Image(int w, int h, int nch): width(w), height(h), nchannels(nch)
{
	buildPixmap();
	filename = channelname = "";
}

Image& Image::operator=(const Image& rhs) 
{
	if (this == &rhs) return *this;
	copy(rhs);
	return *this;
}

void Image::copy(const Image& img)
{
	width  = img.width;
	height = img.height;
	filename  = img.filename;
	nchannels = img.nchannels; 	
	channelname = img.channelname;

	buildPixmap();

	for (int i = 0; i < height; ++i) 
		for (int j = 0; j < width; ++j) 
			for (int channel = 0; channel < RGBA; ++channel)
				setColorValue(i, j, img.colorValue(i, j, channel), channel);
}

/**
 * Use width, height to set up a pixmap
 * Always set pixmap to be black, opaque, use as many channel as needed 
 */
void Image::buildPixmap()
{
	pixmap = new unsigned char[RGBA * width * height];
	for (int i = 0; i < height; ++i) 
		for (int j = 0; j < width; ++j) 
			for (int channel = 0; channel < RGBA; ++channel) 
				pixmap[(i * width + j) * RGBA + channel] = channel == A ? 255 : 0;
}

/**
 * OpenGL treats lower-left as origin, 
 * To display on screen or export image, needs to flip to upper-left first
 */
void Image::flip()
{
	for(int i = height - 1; i > (height +1) / 2 - 1; --i) 
    	for(int j = 0; j < width; ++j) 
    		for (int channel = 0; channel < RGBA; ++channel)
    			std::swap(pixmap[(i * width + j) * RGBA + channel], 
      			  		  pixmap[((height - 1 - i) * width + j) * RGBA + channel]);
}

void Image::tofloat()
{
	if (floatPixmap == NULL) 
		floatPixmap = new float[RGBA * width * height];
	
	for (int i = 0; i < height; ++i)
		for (int j = 0; j < width; ++j)
			for (int channel = 0; channel < RGBA; ++channel)
				floatPixmap[(i * width + j) * RGBA + channel] = (float)colorValue(i, j, channel) / 255.0;
}

/**
 * Convert scaled(0 ~ 1) floatPixmap to range 0 ~ 255
 */
void Image::topixmap()
{
	if (floatPixmap == NULL)
		return;

	unsigned char val = 0;
	for (int i = 0; i < height; ++i) {
		for (int j = 0; j < width; ++j) {
			for (int channel = 0; channel < RGBA; ++channel) {
				val =  (int)floor(floatPixmap[(i * width + j) * RGBA + channel] * 255.0);
				setColorValue(i, j, (val < 0 ? 0 : (val > 255 ? 255 : val)), channel);
			}
		}
	}
}

void Image::toRGB(unsigned char *RGBPixmap, int w, int h)
{
	for (int i = 0; i < h; ++i) 
		for (int j = 0; j < w; ++j) 
			for (int channel = 0; channel < RGB; ++channel)
				RGBPixmap[(i * w + j) * RGB + channel] = colorValue(i, j, channel);
}

void Image::toGRAY(unsigned char *GRAYPixmap, int w, int h)
{
	for (int i = 0; i < h; ++i) 
		for (int j = 0; j < w; ++j) 
			GRAYPixmap[i * w + j] = colorValue(i, j, R);
}

void Image::toRGBA(unsigned char *fromPixmap, int nchannels)
{
	if (nchannels != GRAY && nchannels != RGB) {
		std::cerr << "ERROR: unsupported nchannels = " << nchannels << std::endl;
	}

	for (int i = 0; i < height; ++i) 
		for (int j = 0; j < width; ++j) 
			for (int channel = 0; channel < RGB; ++channel) 
				switch(nchannels) {
					case GRAY:
						setColorValue(i, j, fromPixmap[(i * width + j) * GRAY], channel);
						break;
					case RGB:
						setColorValue(i, j, fromPixmap[(i * width + j) * nchannels + channel], channel);
						break;
					default: break;
				}
}

int Image::getWidth() const
{
	return width;
}

int Image::getHeight() const
{
	return height;
}

int Image::getNchannels() const
{
	return nchannels;
}

std::string Image::getFilename() const
{
	return filename;
}

void Image::setFilename(const std::string n)
{
	filename = n;
}

std::string Image::getChannelname() const
{
	return channelname;
}

unsigned char Image::colorValue(int row, int col, int channel) const
{
	return pixmap[(row * width + col) * RGBA + channel];
}

void Image::setColorValue(int row, int col, unsigned char val, int channel)
{
	pixmap[(row * width + col) * RGBA + channel] = val;
}

bool Image::empty() const
{
	return pixmap == NULL;
}

void Image::print() const
{
	if (pixmap == NULL) {
		std::cout << "ERROR: image is empty" << std::endl;
		return;
	}

	std::cout << "pixmap" << std::endl;
	for (int i = 0; i < height; ++i) {
		for (int j = 0; j < width; ++j) {
			for (int channel = 0; channel < nchannels; ++channel) {
				std::cout << (int)colorValue(i, j, channel) << " ";
			}
			std::cout << std::endl;
		}
	}

	std::cout << "width " << width << " height " << height << " nchannels " << nchannels << std::endl;
	
	if (!filename.empty())
		std::cout << "filename " << filename << std::endl;
	if (!channelname.empty()) 
		std::cout << "channelname" << channelname << std::endl;
}
