##############################################################################
#--------------------------------------------------------------------------
#Copyright (c) 2009, Code Aurora Forum. All rights reserved.

#Redistribution and use in source and binary forms, with or without
#modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of Code Aurora nor
#      the names of its contributors may be used to endorse or promote
#      products derived from this software without specific prior written
#      permission.

#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
#NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
#CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
#EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
#PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
#OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
#WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
#OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
#ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#--------------------------------------------------------------------------
##############################################################################

LOCAL_DIR := $(SRCDIR)/8k/adec-mp3

#-----------------------------------------------------------------------------
#                 Common definitons
#-----------------------------------------------------------------------------

CFLAGS   += $(QCT_CFLAGS)
CPPFLAGS += $(QCT_CPPFLAGS)
CPPFLAGS += -g
CPPFALGS += -D_DEBUG
CPPFLAGS += -I$(LOCAL_DIR)/inc
CPPFLAGS += -I$(SRCDIR)/../mm-core/omxcore/inc
CPPFLAGS += -I$(KERNEL_DIR)/include

#-----------------------------------------------------------------------------
#             Make the Shared library
#-----------------------------------------------------------------------------

vpath %.c $(LOCAL_DIR)/src
vpath %.cpp $(LOCAL_DIR)/src

SRCS := omx_mp3_adec.cpp
SRCS += adec_svr.c

all: libOmxMp3Dec.so.$(LIBVER)

MM_ADEC_OMXMP3_LDLIBS := -lpthread
MM_ADEC_OMXMP3_LDLIBS += -lstdc++

libOmxMp3Dec.so.$(LIBVER): $(SRCS)
	$(CC) $(CPPFLAGS) $(QCT_CFLAGS_SO) $(QCT_LDFLAGS_SO) -Wl,-soname,libOmxMp3Dec.so.$(LIBMAJOR) -o $@ $^ $(MM_ADEC_OMXMP3_LDLIBS)

#-----------------------------------------------------------------------------
#             Make the apps-test (mm-adec-omxmp3-test)
#-----------------------------------------------------------------------------

mm-adec-omxmp3-test: libOmxMp3Dec.so.$(LIBVER)

all: mm-adec-omxmp3-test

vpath %.c $(LOCAL_DIR)/test

MM_ADEC_MP3_TEST_LDLIBS := -lpthread
MM_ADEC_MP3_TEST_LDLIBS += -ldl
MM_ADEC_MP3_TEST_LDLIBS += libOmxMp3Dec.so.$(LIBVER)
MM_ADEC_MP3_TEST_LDLIBS += $(SYSROOT_DIR)/libmm-omxcore.so

TEST_SRCS := omx_mp3_dec_test.c

mm-adec-omxmp3-test: $(TEST_SRCS)
	$(CC) $(CPPFLAGS) $(LDFLAGS) -o $@ $^ $(MM_ADEC_MP3_TEST_LDLIBS)

#-----------------------------------------------------------------------------
#                     END
#-----------------------------------------------------------------------------
