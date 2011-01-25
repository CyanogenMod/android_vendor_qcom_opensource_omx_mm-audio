# sources and intermediate files are separated

AM_CFLAGS = -Wall
AM_CFLAGS += -Wundef
AM_CFLAGS += -Wstrict-prototypes
AM_CFLAGS += -Wno-trigraphs
AM_CFLAGS += -g -O3

AM_CPPFLAGS = -D__packed__=
AM_CPPFLAGS += -DIMAGE_APPS_PROC
AM_CPPFLAGS += -DFEATURE_Q_SINGLE_LINK
AM_CPPFLAGS += -DFEATURE_Q_NO_SELF_QPTR
AM_CPPFLAGS += -DFEATURE_LINUX
AM_CPPFLAGS += -DFEATURE_NATIVELINUX
AM_CPPFLAGS += -DFEATURE_DSM_DUP_ITEMS
AM_CPPFLAGS += -D_DEBUG
AM_CPPFLAGS += -Iinc

c_sources  =src/omx_mp3_adec.cpp
c_sources +=src/adec_svr.c

lib_LTLIBRARIES = libOmxMp3Dec.la
libOmxMp3Dec_la_SOURCES = $(c_sources)
libOmxMp3Dec_la_CFLAGS = $(AM_CFLAGS) -fPIC
libOmxMp3Dec_la_LDLIBS = -lOmxCore -lstdc++ -lpthread
libOmxMp3Dec_la_LDFLAGS = -shared -version-info $(OMXAUDIO_LIBRARY_VERSION)

bin_PROGRAMS = mm-adec-omxmp3-test
mm_adec_omxmp3_test_SOURCES = test/omx_mp3_dec_test.c
mm_adec_omxmp3_test_LDADD = -lOmxCore -ldl -lpthread libOmxMp3Dec.la
