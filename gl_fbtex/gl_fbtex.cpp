/*
 * Copyright (C) 2010 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <sched.h>
#include <fcntl.h>  
#include <unistd.h> 
#include <errno.h> 
#include <sys/resource.h>
#include <sys/ioctl.h> 
#include <sys/mman.h>
#include <linux/fb.h> 

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES/gl.h>
#include <GLES/glext.h>

#include <utils/Timers.h>
#include <ui/FramebufferNativeWindow.h>
#include <ui/GraphicBuffer.h>
#include "EGLUtils.h"

using namespace android;

static int xioctl( int fd,int request,void * arg ) 
{ 
  int r; 
  do
  {
    r = ioctl( fd, request, arg ); 
  }while( -1 == r && EINTR == errno ); 
  return r; 
}

static void printGLString(const char *name, GLenum s) {
  // fprintf(stderr, "printGLString %s, %d\n", name, s);
  const char *v = (const char *) glGetString(s);
  // int error = glGetError();
  // fprintf(stderr, "glGetError() = %d, result of glGetString = %x\n", error,
  //        (unsigned int) v);
  // if ((v < (const char*) 0) || (v > (const char*) 0x10000))
  //    fprintf(stderr, "GL %s = %s\n", name, v);
  // else
  //    fprintf(stderr, "GL %s = (null) 0x%08x\n", name, (unsigned int) v);
  fprintf(stderr, "GL %s = %s\n", name, v);
}

static void checkEglError(const char* op, EGLBoolean returnVal = EGL_TRUE) {
  if (returnVal != EGL_TRUE) {
    fprintf(stderr, "%s() returned %d\n", op, returnVal);
  }

  for (EGLint error = eglGetError(); error != EGL_SUCCESS; error
      = eglGetError()) {
    fprintf(stderr, "after %s() eglError %s (0x%x)\n", op, EGLUtils::strerror(error),
        error);
  }
}

static void checkGlError(const char* op) {
  for (GLint error = glGetError(); error; error
      = glGetError()) {
    fprintf(stderr, "after %s() glError (0x%x)\n", op, error);
  }
}

bool setupGraphics(int w, int h) {
  glViewport(0, 0, w, h);
  checkGlError("glViewport");
  return true;
}

int align(int x, int a) {
  return (x + (a-1)) & (~(a-1));
}

const int fbTexWidth    = 640;
const int fbTexHeight   = 240;
const int fbTexUsage    = GraphicBuffer::USAGE_HW_TEXTURE |
                          GraphicBuffer::USAGE_SW_WRITE_RARELY;
const int fbTexFormat   = HAL_PIXEL_FORMAT_RGB_565;
static sp<GraphicBuffer> fbTexBuffer;
static GLuint fbTex;

int                         fd, 
                            scrSize;
struct  fb_var_screeninfo   vInfo;
struct  fb_fix_screeninfo   fInfo; 
char*                       buf;
unsigned char              *pFbBuf;
unsigned char               color;

void fillFbTexture(EGLDisplay dpy, EGLContext context)
{
  status_t err = fbTexBuffer->lock( GRALLOC_USAGE_SW_WRITE_OFTEN, (void**)(&buf) );
  if (err != 0) 
  {
    fprintf( stderr, "fbTexBuffer->lock(...) failed: %d\n", err );
    return;
  }

  // Get variable screen information. 
  if( -1 == xioctl( fd, FBIOGET_VSCREENINFO, &vInfo ) ) 
  { 
    fprintf( stderr, "Error reading variable information.\n" ); 
  }
  memcpy( buf,
          pFbBuf + ( vInfo.yoffset * vInfo.xres * vInfo.bits_per_pixel / 8 ),
          vInfo.xres * vInfo.yres * vInfo.bits_per_pixel / 8);

  err = fbTexBuffer->unlock();
  if (err != 0) 
  {
    fprintf( stderr, "fbTexBuffer->unlock() failed: %d\n", err );
    return;
  }

  EGLClientBuffer clientBuffer = (EGLClientBuffer)fbTexBuffer->getNativeBuffer();
  EGLImageKHR img = eglCreateImageKHR(  dpy, 
                                        EGL_NO_CONTEXT, 
                                        EGL_NATIVE_BUFFER_ANDROID,
                                        clientBuffer, 
                                        0 );
  checkEglError("eglCreateImageKHR");
  if (img == EGL_NO_IMAGE_KHR) 
  {
    return;
  }

  glGenTextures(1, &fbTex);
  checkGlError("glGenTextures");
  glBindTexture(GL_TEXTURE_EXTERNAL_OES, fbTex);
  checkGlError("glBindTexture");
  glEGLImageTargetTexture2DOES(GL_TEXTURE_EXTERNAL_OES, (GLeglImageOES)img);
  checkGlError("glEGLImageTargetTexture2DOES");
  glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  checkGlError("glTexParameteri");
  glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  checkGlError("glTexParameteri");
  glTexEnvx(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
  checkGlError("glTexEnvx");

  GLint crop[4] = { 0, 0, fbTexWidth, fbTexHeight };
  glTexParameteriv(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_CROP_RECT_OES, crop);
  checkGlError("glTexParameteriv");
}

void openFbDevice(void)
{
  fd = open( "/dev/graphics/fb0", O_RDONLY );
  if( fd < 0 ) 
  {
    fprintf( stderr, "could not open %s, %s\n", "/dev/graphics/fb0", strerror( errno ) );
    return;
  }

  // Get fixed screen information 
  if( -1 == xioctl( fd, FBIOGET_FSCREENINFO, &fInfo ) ) 
  { 
    printf("Error reading fixed information.\n"); 
  }

  // Get variable screen information. 
  if( -1 == xioctl( fd, FBIOGET_VSCREENINFO, &vInfo ) ) 
  { 
    fprintf( stderr, "Error reading variable information.\n" ); 
  } 
  scrSize = vInfo.xres_virtual * vInfo.yres_virtual * vInfo.bits_per_pixel / 8;
  fprintf( stderr, "Visible res:    %dx%d\n", vInfo.xres, vInfo.yres );
  fprintf( stderr, "Virtual res:    %dx%d\n", vInfo.xres_virtual, vInfo.yres_virtual );
  fprintf( stderr, "Offset  res:    %dx%d\n", vInfo.xoffset, vInfo.yoffset );
  fprintf( stderr, "Bits per pixel: %d\n", vInfo.bits_per_pixel );
  fprintf( stderr, "Red:   %d(%d)\n", vInfo.red.offset, vInfo.red.length );
  fprintf( stderr, "Green: %d(%d)\n", vInfo.green.offset, vInfo.green.length );
  fprintf( stderr, "Blue:  %d(%d)\n", vInfo.blue.offset, vInfo.blue.length );
  fprintf( stderr, "Alpha: %d(%d)\n", vInfo.transp.offset, vInfo.transp.length );

  // Map frame buffer device to memory.
  pFbBuf = ( unsigned char * )mmap( NULL, scrSize, PROT_READ, MAP_SHARED , fd, 0 ); 
  if( (int)pFbBuf == -1 ) 
  { 
    fprintf( stderr, "Error: failed to map framebuffer device to memory.\n" ); 
  }
}

void closeFbDevice(void)
{
  munmap( pFbBuf, scrSize );
  close(fd);
}


bool setupFbTexSurface(EGLDisplay dpy, EGLContext context) 
{
  fbTexBuffer = new GraphicBuffer( fbTexWidth, 
                                   fbTexHeight, 
                                   fbTexFormat,
                                   fbTexUsage);
  openFbDevice();
  fillFbTexture(dpy, context);

  return true;
}

void renderFrame(int w, int h) {
  glClearColor(0.0f, 0.0f, 1.0f, 1.0f);
  checkGlError("glClearColor");
  glClear( GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
  checkGlError("glClear");
  
  glBindTexture(GL_TEXTURE_EXTERNAL_OES, fbTex);
  checkGlError("glBindTexture");
  glEnable(GL_TEXTURE_EXTERNAL_OES);
  checkGlError("glEnable");

  glDrawTexiOES(0, 0, 0, w, h);
  checkGlError("glDrawTexiOES");
}

void printEGLConfiguration(EGLDisplay dpy, EGLConfig config) {

#define X(VAL) {VAL, #VAL}
  struct {EGLint attribute; const char* name;} names[] = {
  X(EGL_BUFFER_SIZE),
  X(EGL_ALPHA_SIZE),
  X(EGL_BLUE_SIZE),
  X(EGL_GREEN_SIZE),
  X(EGL_RED_SIZE),
  X(EGL_DEPTH_SIZE),
  X(EGL_STENCIL_SIZE),
  X(EGL_CONFIG_CAVEAT),
  X(EGL_CONFIG_ID),
  X(EGL_LEVEL),
  X(EGL_MAX_PBUFFER_HEIGHT),
  X(EGL_MAX_PBUFFER_PIXELS),
  X(EGL_MAX_PBUFFER_WIDTH),
  X(EGL_NATIVE_RENDERABLE),
  X(EGL_NATIVE_VISUAL_ID),
  X(EGL_NATIVE_VISUAL_TYPE),
  X(EGL_SAMPLES),
  X(EGL_SAMPLE_BUFFERS),
  X(EGL_SURFACE_TYPE),
  X(EGL_TRANSPARENT_TYPE),
  X(EGL_TRANSPARENT_RED_VALUE),
  X(EGL_TRANSPARENT_GREEN_VALUE),
  X(EGL_TRANSPARENT_BLUE_VALUE),
  X(EGL_BIND_TO_TEXTURE_RGB),
  X(EGL_BIND_TO_TEXTURE_RGBA),
  X(EGL_MIN_SWAP_INTERVAL),
  X(EGL_MAX_SWAP_INTERVAL),
  X(EGL_LUMINANCE_SIZE),
  X(EGL_ALPHA_MASK_SIZE),
  X(EGL_COLOR_BUFFER_TYPE),
  X(EGL_RENDERABLE_TYPE),
  X(EGL_CONFORMANT),
   };
#undef X

  for (size_t j = 0; j < sizeof(names) / sizeof(names[0]); j++) {
    EGLint value = -1;
    EGLint returnVal = eglGetConfigAttrib(dpy, config, names[j].attribute, &value);
    EGLint error = eglGetError();
    if (returnVal && error == EGL_SUCCESS) {
      printf(" %s: ", names[j].name);
      printf("%d (0x%x)\n", value, value);
    }
  }
  printf("\n");
}

int main(int argc, char** argv) {
  EGLBoolean returnValue;
  EGLConfig myConfig = {0};

  EGLint context_attribs[] = { EGL_CONTEXT_CLIENT_VERSION, 1, EGL_NONE };
  EGLint s_configAttribs[] = {
      EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
      EGL_RENDERABLE_TYPE, EGL_OPENGL_ES_BIT,
      EGL_NONE };
  EGLint majorVersion;
  EGLint minorVersion;
  EGLContext context;
  EGLSurface surface;
  EGLint w, h;

  EGLDisplay dpy;

  checkEglError("<init>");
  dpy = eglGetDisplay(EGL_DEFAULT_DISPLAY);
  checkEglError("eglGetDisplay");
  if (dpy == EGL_NO_DISPLAY) {
    printf("eglGetDisplay returned EGL_NO_DISPLAY.\n");
    return 0;
  }

  returnValue = eglInitialize(dpy, &majorVersion, &minorVersion);
  checkEglError("eglInitialize", returnValue);
  fprintf(stderr, "EGL version %d.%d\n", majorVersion, minorVersion);
  if (returnValue != EGL_TRUE) {
    printf("eglInitialize failed\n");
    return 0;
  }

  EGLNativeWindowType window = android_createDisplaySurfaceEx("fb4");
  returnValue = EGLUtils::selectConfigForNativeWindow(dpy, s_configAttribs, window, &myConfig);
  if (returnValue) {
    printf("EGLUtils::selectConfigForNativeWindow() returned %d", returnValue);
    return 1;
  }

  checkEglError("EGLUtils::selectConfigForNativeWindow");

  printf("Chose this configuration:\n");
  printEGLConfiguration(dpy, myConfig);

  surface = eglCreateWindowSurface(dpy, myConfig, window, NULL);
  checkEglError("eglCreateWindowSurface");
  if (surface == EGL_NO_SURFACE) {
    printf("gelCreateWindowSurface failed.\n");
    return 1;
  }

  context = eglCreateContext(dpy, myConfig, EGL_NO_CONTEXT, context_attribs);
  checkEglError("eglCreateContext");
  if (context == EGL_NO_CONTEXT) {
    printf("eglCreateContext failed\n");
    return 1;
  }
  returnValue = eglMakeCurrent(dpy, surface, surface, context);
  checkEglError("eglMakeCurrent", returnValue);
  if (returnValue != EGL_TRUE) {
    return 1;
  }
  eglQuerySurface(dpy, surface, EGL_WIDTH, &w);
  checkEglError("eglQuerySurface");
  eglQuerySurface(dpy, surface, EGL_HEIGHT, &h);
  checkEglError("eglQuerySurface");
  GLint dim = w < h ? w : h;

  fprintf(stderr, "Window dimensions: %d x %d\n", w, h);

  printGLString("Version", GL_VERSION);
  printGLString("Vendor", GL_VENDOR);
  printGLString("Renderer", GL_RENDERER);
  printGLString("Extensions", GL_EXTENSIONS);

/*
  if(!setupYuvTexSurface(dpy, context)) {
    fprintf(stderr, "Could not set up texture surface.\n");
    return 1;
  }
*/

  if(!setupFbTexSurface(dpy, context)) {
    fprintf(stderr, "Could not set up texture surface.\n");
    return 1;
  }

  if(!setupGraphics(w, h)) {
    fprintf(stderr, "Could not set up graphics.\n");
    return 1;
  }

  color = 50;

  for (;;) {
    static int dir = -1;

    renderFrame(w, h);
    eglSwapBuffers(dpy, surface);
    checkEglError("eglSwapBuffers");
    
    if( (h % 2) == 0 )
    {
      fillFbTexture(dpy, context);
    }

    if (w <= 10 || h <= 10)
    {
      dir = -dir;
    }

    if (w >= 1300 || h >= 900)
    {
      dir = -dir;
    }


    w += dir;
    h += dir;
  }

  return 0;
}
