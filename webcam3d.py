# need pyusb and python binding for opencv
# also need libusb installed
import cv2
import usb.core
import time
import threading


def isnum(num):
    try:
        float(num)
        return True
    except:
        return False


class Camera3d(threading.Thread):

    def __init__(self, cam_usb, cam_id):
        threading.Thread.__init__(self)
        self.cam_usb = cam_usb
        self.cam_id = cam_id
        
    def run(self):
        self.capture()

    def capture(self):
        cap = cv2.VideoCapture(self.cam_id)
        print('capture on interface: {}, is opened: {}'.format(self.cam_id, cap.isOpened()))
        w, h = cap.get(3), cap.get(4)
        print('capture width: {} height: {}'.format(w,h))
        # Define the codec and create VideoWriter object
        #fourcc = cv2.VideoWriter_fourcc(*'XVID')
        #writer = cv2.VideoWriter('output.avi',fourcc, 20.0, (640,480))

        stime = time.time()      
        cv2.namedWindow('webcam', cv2.WINDOW_NORMAL)
        while(True):
            # Capture frame-by-frame
            succeed, frame = cap.read()

            # Our operations on the frame come here
            #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            image = frame # gray
            #image = cv2.flip(image, 0)
                
            # Display the resulting frame
            if succeed:
                print('{:.2f}'.format(time.time()-stime))
                #writer.write(image)
                cv2.imshow('webcam', image)
            else:
                print('failed to capture frame')
                break

            key = cv2.waitKey(1000)
            if key in range(ord('1'),ord('5')):
                para = int(chr(key))
                self.cam_usb.switch(self.cam_id, para)
            elif key == ord('q'):
                break

        # When everything done, release the capture
        cap.release()
        cv2.destroyAllWindows()


class CameraUsb(object):

    def __init__(self, vendor, product):
        self.dev = usb.core.find(idVendor=vendor, idProduct=product)
        self.dev_intf = []

    def deattach(self, intf):
        if self.dev.is_kernel_driver_active(intf):
            print('de-attach from intf: {}'.format(intf))
            self.dev.detach_kernel_driver(intf)
            usb.util.claim_interface(self.dev, intf)            

    def attach(self, intf):
        if not self.dev.is_kernel_driver_active(intf):
            print('attach to intf: {}'.format(intf))
            #time.sleep(1)
            usb.util.release_interface(self.dev, intf)            
            self.dev.attach_kernel_driver(intf)

    def configure(self):
        print('configure SET_CUR sequence??')
        # simulate the SET_CUR sequence
        self.dev.ctrl_transfer(0x21,0x01,0x0800,0x0600,[0x50,0xff])
        self.dev.ctrl_transfer(0x21,0x01,0x0f00,0x0600,[0x00,0xf6])
        self.dev.ctrl_transfer(0x21,0x01,0x0800,0x0600,[0x25,0x00])
        self.dev.ctrl_transfer(0x21,0x01,0x0800,0x0600,[0x5f,0xfe])
        self.dev.ctrl_transfer(0x21,0x01,0x0f00,0x0600,[0x00,0x03])
        self.dev.ctrl_transfer(0x21,0x01,0x0f00,0x0600,[0x00,0x02])
        self.dev.ctrl_transfer(0x21,0x01,0x0f00,0x0600,[0x00,0x12])
        self.dev.ctrl_transfer(0x21,0x01,0x0f00,0x0600,[0x00,0x04])
        self.dev.ctrl_transfer(0x21,0x01,0x0800,0x0600,[0x76,0xc3])

    def switch(self, intf, para):
        print('switch camera mode: {}'.format(para))
        assert(isnum(intf) and isnum(para))
        #for intf in self.dev_intf:
        self.deattach(intf)
        self.configure()
        req = self.dev.ctrl_transfer(0x21,0x01,0x0a00,0x0600,[para,0x00])
        print('usb ctrl xfer ack: {}'.format(req))
        #for intf in self.dev_intf:
        self.attach(intf)

    '''
propId –

Property identifier. It can be one of the following:

    CV_CAP_PROP_POS_MSEC Current position of the video file in milliseconds or video capture timestamp.
    CV_CAP_PROP_POS_FRAMES 0-based index of the frame to be decoded/captured next.
    CV_CAP_PROP_POS_AVI_RATIO Relative position of the video file: 0 - start of the film, 1 - end of the film.
    CV_CAP_PROP_FRAME_WIDTH Width of the frames in the video stream.
    CV_CAP_PROP_FRAME_HEIGHT Height of the frames in the video stream.
    CV_CAP_PROP_FPS Frame rate.
    CV_CAP_PROP_FOURCC 4-character code of codec.
    CV_CAP_PROP_FRAME_COUNT Number of frames in the video file.
    CV_CAP_PROP_FORMAT Format of the Mat objects returned by retrieve() .
    CV_CAP_PROP_MODE Backend-specific value indicating the current capture mode.
    CV_CAP_PROP_BRIGHTNESS Brightness of the image (only for cameras).
    CV_CAP_PROP_CONTRAST Contrast of the image (only for cameras).
    CV_CAP_PROP_SATURATION Saturation of the image (only for cameras).
    CV_CAP_PROP_HUE Hue of the image (only for cameras).
    CV_CAP_PROP_GAIN Gain of the image (only for cameras).
    CV_CAP_PROP_EXPOSURE Exposure (only for cameras).
    CV_CAP_PROP_CONVERT_RGB Boolean flags indicating whether images should be converted to RGB.
    CV_CAP_PROP_WHITE_BALANCE_U The U value of the whitebalance setting (note: only supported by DC1394 v 2.x backend currently)
    CV_CAP_PROP_WHITE_BALANCE_V The V value of the whitebalance setting (note: only supported by DC1394 v 2.x backend currently)
    CV_CAP_PROP_RECTIFICATION Rectification flag for stereo cameras (note: only supported by DC1394 v 2.x backend currently)
    CV_CAP_PROP_ISO_SPEED The ISO speed of the camera (note: only supported by DC1394 v 2.x backend currently)
    CV_CAP_PROP_BUFFERSIZE Amount of frames stored in internal buffer memory (note: only supported by DC1394 v 2.x backend currently)

    '''
 
    def enum(self):
        dev_intf = []
        for config in self.dev:
            print('enum interface num: {}'.format(config.bNumInterfaces))
            for i in range(config.bNumInterfaces):
                if self.dev.is_kernel_driver_active(i):
                    print('  found active intf: {}'.format(i))
                    dev_intf.append(i)
        self.dev_intf = dev_intf
        return self.dev_intf

    def enum_prop(self):
        for i in self.dev_intf:
            cap = cv2.VideoCapture(i)
            for prop_id in range(22):
                print('[{}] feature {} : {}'.format(i, prop_id, cap.get(prop_id)))

import sys

if __name__ == '__main__':
    if len(sys.argv) > 1:
        para = int(sys.argv[1])
    camusb = CameraUsb(0x18e3, 0x5031)
    usbintf = camusb.enum()
    for intf in usbintf:
        cam = Camera3d(camusb, intf)
        #cam.start()
        cam.run()



