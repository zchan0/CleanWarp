CC	= g++
C		= cpp
H   = h

CFLAGS	= -g 
LFLAGS	= -g

ifeq ("$(shell uname)", "Darwin")
  LDFLAGS     = -framework Foundation -framework GLUT -framework OpenGL -lOpenImageIO -lm
else
  ifeq ("$(shell uname)", "Linux")
    LDFLAGS   = -L /usr/lib64/ -lglut -lGL -lGLU -lOpenImageIO -lm
  endif
endif

EXE			= okwarp
HFILES  = ImageIO/ImageIO.${H} ImageIO/Image.${H} 
OBJS    = ImageIO/ImageIO.o ImageIO/Image.o 

all:	${EXE}

okwarp:	${OBJS} okwarp.o
	${CC} ${LFLAGS} -o okwarp ${OBJS} okwarp.o ${LDFLAGS}

okwarp.o: okwarp.${C} ${HFILES}
	${CC} ${CFLAGS} -c okwarp.${C}

Image.o: ImageIO/Image.${C} ${HFILES}
	${CC} ${CFLAGS} -c Image.${C}

ImageIO.o: ImageIO/ImageIO.${C} ${HFILES}
	${CC} ${CFLAGS} -c ImageIO.${C} 

clean:
	rm -f core.* *.o *~ ${EXE} ${OBJS}
