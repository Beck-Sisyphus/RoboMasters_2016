#include "network.h"
#include "detection_layer.h"
#include "cost_layer.h"
#include "utils.h"
#include "parser.h"
#include "box.h"
#include "image.h"
#include "demo.h"
#include <sys/time.h>
#include <math.h>

#define FRAMES 3

#ifdef OPENCV
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/imgproc/imgproc_c.h"
void convert_detections(float *predictions, int classes, int num, int square, int side, int w, int h, float thresh, float **probs, box *boxes, int only_objectness);

static char **demo_names;
static image *demo_labels;
static int demo_classes;

static float **probs;
static box *boxes;
static network net;
static image in   ;
static image in_s ;
static image det  ;
static image det_s;
static image disp = {0};
static CvCapture * cap;
static float fps = 0;
static float demo_thresh = 0;
static int showVideo;

static float *predictions[FRAMES];
static int demo_index = 0;
static image images[FRAMES];
static float *avg;
struct Output {
   int class;
   float x;
   float y;
   float width;
   float height;
   float prob;
} output;  
int find_index(int a[], int num_elements, int value)
{
   int i;
   for (i=0; i<num_elements; i++)
   {
	 if (a[i] == value)
	 {
	    return(value);  /* it was found */
	 }
   }
   return(-1);  /* if it was not found */
}
void printDisAndAngle(struct Output output){
	if(output.prob > 0){
		float ratioWHAt1m = 0.16;
		float ratioDoubleDistance = 2;
		float XDistanceFromCenter = output.x - 0.5;
 		float YDistanceFromCenter = output.y - 0.5;
		printf("---\n");
        	if(output.class == 0 || output.class == 2){
			printf("red\n");	
		}else{
			printf("blue\n");
		}
		double distance = pow(2, log(ratioWHAt1m/(output.width*output.height))/log(ratioDoubleDistance));
		printf("%f\n", 1.35*distance);
		float valueY =  XDistanceFromCenter * 1.35*distance;                
		printf("%f\n", atan(valueY));
                float valueZ =  YDistanceFromCenter * 1.35*distance;   
		printf("%f\n", atan(valueZ));
	}
}
void *fetch_in_thread(void *ptr)
{
    in = get_image_from_stream(cap);
    if(!in.data){
        error("Stream closed.");
    }
    in_s = resize_image(in, net.w, net.h);
    return 0;
}
void draw_detections2(image im, int num, float thresh, box *boxes, float **probs, char **names, image *labels, int classes)
{
    int i;
    int redArmor = 0;
    int blueArmor = 0;
    for(i = 0; i < num; ++i){
    	int class = max_index(probs[i], classes);
        float prob = probs[i][class];
        if(prob > thresh && class == 2){
		printf("1!!");
		redArmor = 1;	
	}else if(prob > thresh && class == 3){
		blueArmor = 1;
	}
    }
    //printf("%d %d\n", redArmor, blueArmor);
    int exclude[2];
    exclude[0] = -10;
    exclude[1] = -10;
    if (redArmor == 1){
    	exclude[0] = 0;
    }else if(blueArmor == 1){
    	exclude[1] = 1;
    }
    struct Output redOutput;
    struct Output blueOutput;
    redOutput.prob = -1;
    blueOutput.prob = -1;   
    for(i = 0; i < num; ++i){
        int class = max_index(probs[i], classes);
        //printf("%d  %d\n", exclude[0], exclude[1]);
	if(find_index(exclude, 2, class) == -1){
		float prob = probs[i][class];
        	if(prob > thresh && (class == 0 || class == 2)&& prob > redOutput.prob ){
			box b = boxes[i];
            		redOutput.prob = prob;
			redOutput.x = b.x;
			redOutput.y =b.y;
			redOutput.width = b.w;
			redOutput.height = b.h;
                        redOutput.class = class; 

        	} else if(prob > thresh && (class == 1 || class == 3)&& prob > blueOutput.prob ){
			box b = boxes[i];
            		blueOutput.prob = prob;
			blueOutput.x = b.x;
			blueOutput.y =b.y;
			blueOutput.width = b.w;
			blueOutput.height = b.h;
                        blueOutput.class = class; 
		}
	}
    }
    printDisAndAngle(redOutput);
    printDisAndAngle(blueOutput);	
}


void *detect_in_thread(void *ptr)
{
    float nms = .4;

    detection_layer l = net.layers[net.n-1];
    float *X = det_s.data;
    float *prediction = network_predict(net, X);

    memcpy(predictions[demo_index], prediction, l.outputs*sizeof(float));
    mean_arrays(predictions, FRAMES, l.outputs, avg);

    free_image(det_s);
    convert_detections(avg, l.classes, l.n, l.sqrt, l.side, 1, 1, demo_thresh, probs, boxes, 0);
    if (nms > 0) do_nms(boxes, probs, l.side*l.side*l.n, l.classes, nms);
    images[demo_index] = det;
    det = images[(demo_index + FRAMES/2 + 1)%FRAMES];
    demo_index = (demo_index + 1)%FRAMES;
    if(showVideo == 0){
    	draw_detections(det, l.side*l.side*l.n, demo_thresh, boxes, probs, demo_names, demo_labels, demo_classes);
    }
    draw_detections2(det, l.side*l.side*l.n, 0.25, boxes, probs, demo_names, demo_labels, demo_classes);

    return 0;
}


double get_wall_time()
{
    struct timeval time;
    if (gettimeofday(&time,NULL)){
        return 0;
    }
    return (double)time.tv_sec + (double)time.tv_usec * .000001;
}

void demo(char *cfgfile, char *weightfile, float thresh, int cam_index, const char *filename, char **names, image *labels, int classes, int frame_skip, int debug)
{
    //skip = frame_skip;
    showVideo = debug;
    int delay = frame_skip;
    demo_names = names;
    demo_labels = labels;
    demo_classes = classes;
    demo_thresh = thresh;
    //printf("Demo\n");
    //net = parse_network_cfg(cfgfile);
    net = parse_network_cfg(cfgfile);
    load_weights(&net, weightfile);
    set_batch_network(&net, 1);

    srand(2222222);

    if(filename){
        cap = cvCaptureFromFile(filename);
    }else{
        cap = cvCaptureFromCAM(cam_index);
    }

    if(!cap) error("Couldn't connect to webcam.\n");

    detection_layer l = net.layers[net.n-1];
    int j;

    avg = (float *) calloc(l.outputs, sizeof(float));
    for(j = 0; j < FRAMES; ++j) predictions[j] = (float *) calloc(l.outputs, sizeof(float));
    for(j = 0; j < FRAMES; ++j) images[j] = make_image(1,1,3);

    boxes = (box *)calloc(l.side*l.side*l.n, sizeof(box));
    probs = (float **)calloc(l.side*l.side*l.n, sizeof(float *));
    for(j = 0; j < l.side*l.side*l.n; ++j) probs[j] = (float *)calloc(l.classes, sizeof(float *));

    pthread_t fetch_thread;
    pthread_t detect_thread;

    fetch_in_thread(0);
    det = in;
    det_s = in_s;

    fetch_in_thread(0);
    detect_in_thread(0);
    disp = det;
    det = in;
    det_s = in_s;

    for(j = 0; j < FRAMES/2; ++j){
        fetch_in_thread(0);
        detect_in_thread(0);
        disp = det;
        det = in;
        det_s = in_s;
    }

    int count = 0;
    if(showVideo == 1){
    	cvNamedWindow("Demo", CV_WINDOW_NORMAL); 
    	cvMoveWindow("Demo", 0, 0);
    	cvResizeWindow("Demo", 1352, 1013);
    }

    double before = get_wall_time();

    while(1){
        ++count;
        if(1){
            if(pthread_create(&fetch_thread, 0, fetch_in_thread, 0)) error("Thread creation failed");
            if(pthread_create(&detect_thread, 0, detect_in_thread, 0)) error("Thread creation failed");

            show_image(disp, "Demo");
            int c = cvWaitKey(1);
            if (c == 10){
                if(frame_skip == 0) frame_skip = 60;
                else if(frame_skip == 4) frame_skip = 0;
                else if(frame_skip == 60) frame_skip = 4;   
                else frame_skip = 0;
            }

            pthread_join(fetch_thread, 0);
            pthread_join(detect_thread, 0);

            if(delay == 0){
                free_image(disp);
                disp  = det;
            }
            det   = in;
            det_s = in_s;
        }else {
            fetch_in_thread(0);
            det   = in;
            det_s = in_s;
            detect_in_thread(0);
            if(delay == 0) {
                free_image(disp);
                disp = det;
            }
            show_image(disp, "Demo");
            cvWaitKey(1);
        }
        --delay;
        if(delay < 0){
            delay = frame_skip;

            double after = get_wall_time();
            float curr = 1./(after - before);
            fps = curr;
            before = after;
        }
    }
}
#else
void demo(char *cfgfile, char *weightfile, float thresh, int cam_index, const char *filename, char **names, image *labels, int classes, int frame_skip)
{
    fprintf(stderr, "Demo needs OpenCV for webcam images.\n");
}
#endif

