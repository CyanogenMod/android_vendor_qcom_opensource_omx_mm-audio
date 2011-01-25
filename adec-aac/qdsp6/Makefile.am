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

c_sources  =src/omx_aac_adec.cpp
c_sources +=src/adec_svr.c

lib_LTLIBRARIES = libOmxAacDec.la
libOmxAacDec_la_SOURCES = $(c_sources)
libOmxAacDec_la_CFLAGS = $(AM_CFLAGS) -fPIC
libOmxAacDec_la_LDLIBS = -lOmxcore -lstdc++ -lpthread
libOmxAacDec_la_LDFLAGS = -shared -version-info $(OMXAUDIO_LIBRARY_VERSION)

bin_PROGRAMS = mm-adec-omxaac-test
mm_adec_omxaac_test_SOURCES = test/omx_aac_dec_test.c
mm_adec_omxaac_test_LDADD = -lOmxCore -ldl -lpthread libOmxAacDec.la
