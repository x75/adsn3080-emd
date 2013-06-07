
CC = /usr/bin/gcc

# CFLAGS = -I../mavlink/include/huch
CFLAGS = -I../mavhub/thirdparty/mavlink/v1.0/huch

all: ADNS3080-getmotion

ADNS3080-getmotion: ADNS3080-getmotion.c

clean:
	rm -v ADNS3080-getmotion
