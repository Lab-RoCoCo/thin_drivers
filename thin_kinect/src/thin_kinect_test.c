#include <string.h>
#include <stdio.h>
#include <signal.h>

#include "libfreenect.h"
#include <pthread.h> 


typedef struct freenect_thread_params {
  volatile run;
  int device_num;
  freenect_frame_mode video_mode;
  freenect_frame_mode depth_mode;
} freenect_thread_params;

static freenect_thread_params params;

void handle_sigint(int i) {
  params.run = 0;
}

void* freenect_thread(void* v) {
  freenect_thread_params* params = (freenect_thread_params*) v;
  int retval = 0;
  while (params->run && ! retval) {
    char* video_buffer, *depth_buffer;
    uint32_t video_ts;
    uint32_t depth_ts;
    if (params->video_mode.video_format != FREENECT_VIDEO_DUMMY) 
      retval = freenect_sync_get_video(&video_buffer, 
				       &video_ts, 
				       params->device_num, 
				       params->video_mode.video_format);
    if (retval < 0) {
      printf("error in getting rgb: %d\n", retval);
      break;
    } else
      printf("c");

    if (params->depth_mode.depth_format != FREENECT_DEPTH_DUMMY) 
      retval = freenect_sync_get_depth(&video_buffer, 
				       &depth_ts, 
				       params->device_num, 
				       params->depth_mode.depth_format);
    if (retval < 0) {
      printf("error in getting depth: %d\n", retval);
      break;
    } else
      printf("d");
    fflush(stdout);
  }
  freenect_sync_stop();
  
}

int main(int argc, char** argv) {
  signal(SIGINT, handle_sigint);
  
  int device_num=-1;
  int depth_mode=-1;
  int video_mode=-1;

  if (argc<4) {
    printf ("usage: %s <device_num> <depth_mode> <rgb_mode>\n", argv[0]);
    printf ("set depth_mode=-1 and rgb_mode=-1 for a list of modes");
    device_num = -1;
  } else {
    device_num = atoi(argv[1]);
    depth_mode = atoi(argv[2]);
    video_mode = atoi(argv[3]);
  }

  //freenect STUFF
  freenect_context* context;
  int retval = freenect_init(&context,NULL);
  if (retval<0) {
    printf("error in initializing freenect %d \n", retval);
    return 0;
  }

  // select only the camera
  freenect_select_subdevices(context, FREENECT_DEVICE_CAMERA);

  //enumerate the devices
  int num_devices = freenect_num_devices(context);
  if (num_devices<=0) {
    printf("no devices found\n");
    return 0;
  }  else {
    printf("found %d devices\n", num_devices);
  }

  // obtain the list of attributes for each device
  struct freenect_device_attributes* attribute_list;
  retval = freenect_list_device_attributes(context, &attribute_list);
  if (retval<0) {
    printf("error getting attribute list %d \n", retval);
    return 0;
  }
  
  struct freenect_device_attributes* aux = attribute_list;
  int k = 0;
  while (aux) {
    printf("device: %02d, serial %s\n", k, aux->camera_serial);
    aux = aux->next;
    k++;
  }
  

  int num_video_modes = freenect_get_video_mode_count();
  printf ("found %d video modes\n", num_video_modes);
  int i;
  for (i = 0; i< num_video_modes; i++){
    const char* fmt = 0;
    freenect_frame_mode mode = freenect_get_video_mode(i);
    switch(mode.video_format) {
    case FREENECT_VIDEO_RGB: fmt = "FREENECT_VIDEO_RGB"; break;
    case FREENECT_VIDEO_BAYER: fmt = "FREENECT_VIDEO_BAYER"; break;
    case FREENECT_VIDEO_IR_8BIT: fmt = "FREENECT_VIDEO_IR_8BIT"; break;
    case FREENECT_VIDEO_IR_10BIT: fmt = "FREENECT_VIDEO_IR_10BIT"; break;
    case FREENECT_VIDEO_IR_10BIT_PACKED: fmt = "FREENECT_VIDEO_IR_10BIT_PACKED"; break;
    case FREENECT_VIDEO_YUV_RGB: fmt = "FREENECT_VIDEO_YUV_RGB"; break;
    case FREENECT_VIDEO_YUV_RAW: fmt = "FREENECT_VIDEO_YUV_RAW"; break;
    }

    printf("mode: %02d, format: %s, width: %d, height: %d, bpp: %d, padding: %d, framerate: %d, valid: %d, bytes: %d\n",
     i, 
     fmt, mode.width, 
     mode.height, 
     mode.data_bits_per_pixel, 
     mode.padding_bits_per_pixel, 
     mode.framerate,
     mode.is_valid,
     mode.bytes);

  }


  int num_depth_modes = freenect_get_depth_mode_count();
  printf ("found %d depth modes\n", num_depth_modes);
  for (i = 0; i< num_depth_modes; i++){
    const char* fmt = 0;
    freenect_frame_mode mode = freenect_get_depth_mode(i);
    switch(mode.depth_format) {
    case FREENECT_DEPTH_11BIT: fmt = "FREENECT_DEPTH_11BIT"; break;
    case FREENECT_DEPTH_10BIT: fmt = "FREENECT_DEPTH_10BIT"; break;
    case FREENECT_DEPTH_11BIT_PACKED: fmt = "FREENECT_DEPTH_11BIT_PACKED"; break;
    case FREENECT_DEPTH_10BIT_PACKED: fmt = "FREENECT_DEPTH_10BIT_PACKED"; break;
    case FREENECT_DEPTH_REGISTERED: fmt = "FREENECT_DEPTH_REGISTERED"; break;
    case FREENECT_DEPTH_MM: fmt = "FREENECT_DEPTH_MM"; break;
    }

    printf("mode: %02d, format: %s, width: %d, height: %d, bpp: %d, padding: %d, framerate: %d, valid: %d, bytes: %d\n",
     i, 
     fmt, mode.width, 
     mode.height, 
     mode.data_bits_per_pixel, 
     mode.padding_bits_per_pixel, 
     mode.framerate,
     mode.is_valid,
     mode.bytes);

  }


  if (device_num>k || device_num<0){
    printf("device number %d  does not exist, aborting \n", device_num);
    return 0;
  }
  


  char* video_buffer, *depth_buffer;

  if (video_mode >num_video_modes || video_mode<0)
    params.video_mode.video_format = FREENECT_VIDEO_DUMMY;
  else  {
    params.video_mode = freenect_get_video_mode(video_mode);
  }

  if (depth_mode >num_depth_modes || depth_mode<0)
    params.depth_mode.depth_format = FREENECT_DEPTH_DUMMY;
  else {
    params.depth_mode = freenect_get_depth_mode(depth_mode);
  }

  params.run = 1;
  params.device_num = device_num;
  pthread_t runner;
  pthread_create(&runner, 0, freenect_thread, (void*) &params);
  pause();
}
