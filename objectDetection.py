import cv2 as cv
import numpy as np
import sys
from serialcom import Serial
from scipy.spatial import distance as dist
from collections import OrderedDict

class Detector(object):
    __SCREEN_WIDTH = 320
    __SCREEN_HEIGHT = 240

    
    def __init__(self, width=320, height=240, model=r"classifiers/testcascade2.xml"):
        #initialize variables for cv2
        self.width = width
        self.height = height
        self.font = cv.FONT_HERSHEY_COMPLEX_SMALL
        self.fontScale = 1
        self.fontColor1 = (255,0,0)
        self.fontColor2 = (0,255,0)
        self.lineType = 2
        self.boxLineWidth = 3
        self.circleRadius = 5
        self.ref_area = 60
        self.scale_factor = 1.20
        self.neighbor = 4
        
        # load model
        self.cascade = cv.CascadeClassifier(model)

        # init camera
        self.camera = cv.VideoCapture(0)
        self.camera.set(3, self.__SCREEN_WIDTH)
        self.camera.set(4, self.__SCREEN_HEIGHT)

        # check if camera detected
        if not self.camera.isOpened():
            print("Error loading video capture device")
            exit()

        # initialize serial variable
        self.ser = Serial()

        # intialize centroid vairable
        self.ct = centroidTracker()

    def detectObject(self, frame):
        #process frame and draw boxes
        gray_scale_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        hsv_frame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        lower_yellow = np.array([25,100,100])
        upper_yellow = np.array([95,255,255])
        self.mask = cv.inRange(hsv_frame, lower_yellow, upper_yellow)
        self.res = cv.bitwise_and(frame, frame, mask=self.mask)
        self.res = cv.GaussianBlur(self.res, (15, 15), 0)
        detection = self.cascade.detectMultiScale(self.res, 
        self.scale_factor, self.neighbor)
        
        self.centroids = []
        for (x,y,w,h) in detection:
            # center point
            cx = int(x+(w/2))
            cy = int(y+(h/2))

            self.centroid = (cx,cy)
            self.centroids.append(self.centroid)
        
        # call centroid update method to get an ordered dictionary containing 
        # the IDs and coordinates
        objects_ = self.ct.update(self.centroids)

        if objects_:
            # get tuple with highest y meaning it's the closest ball
            self.closest_ball_coord = max(y[1] for y in objects_.values())
            
            # get the id of that ball
            self.ball_id = [id_ for id_, coord in objects_.items() 
            if coord[1] == self.closest_ball_coord][0]
            

        for (objectID, centroid) in objects_.items():
            # draw both the ID of the object and the centroid of the
            # object on the output frame
            text = "ID {}".format(objectID)
            cv.putText(frame, text, (centroid[0] - 10, centroid[1] - 10),
            self.font, self.fontScale, self.fontColor1, self.boxLineWidth)
            cv.circle(frame, (centroid[0], centroid[1]), self.circleRadius,
            self.fontColor1,-1)
        
        # send to the arduino the coords of the closest ball
        self.ser.send_data(objects_[self.ball_id])

        return frame


    def detect(self):
        # read frames from camera
        while self.camera.isOpened():
            _, frame = self.camera.read()
            self.processed_image = self.detectObject(frame)
            cv.imshow("frame", frame)
            cv.imshow("mask", self.res)

            if cv.waitKey(1) == ord("q"):
                self.camera.release()
                cv.destroyAllWindows()
                break


class centroidTracker(object):
    def __init__(self, maxFramesRM=10):
        self.nextObjectID = 0
        # Store ID and centroid coordinates in dict (key:value)
        self.objects = OrderedDict()
        self.disappeared = OrderedDict()
        self.maxFramesRM = maxFramesRM


    def register(self, centroid):
        self.objects[self.nextObjectID] = centroid
        self.disappeared[self.nextObjectID] = 0
        self.nextObjectID += 1


    def deregister(self, objectID):
        del self.objects[objectID]
        del self.disappeared[objectID]


    def update(self, centroids):
        # check to see if the list of centroids
        # is empty
        if len(centroids) == 0:
            # loop over any existing tracked objects and mark them
            # as disappeared
            self.cp_disappeared = self.disappeared.copy()
            for objectID in self.cp_disappeared.keys():
                self.disappeared[objectID] += 1

                # if we have reached a maximum number of consecutive
                # frames where a given object has been marked as
                # missing, deregister it
                if self.disappeared[objectID] > self.maxFramesRM:
                    self.deregister(objectID)

            # return early as there are no centroids or tracking info
            # to update
            return self.objects

    
        # if we are currently not tracking any objects take the input
        # centroids and register each of them
        if len(self.objects) == 0:
            for i in range(0, len(centroids)):
                self.register(centroids[i])

        # otherwise, we are currently tracking objects so we need to
        # try to match the input centroids to existing object
        # centroids
        else:
            # grab the set of object IDs and corresponding centroids
            objectIDs = list(self.objects.keys())
            objectCentroids = list(self.objects.values())

            # compute the distance between each pair of object
            # centroids and input centroids, respectively -- our
            # goal will be to match an input centroid to an existing
            # object centroid
            #modulus of a vector*, first tuple from objecCentroids with all the centroids,
            #second tuple from objecCentroids with all the centroids and so on...
            D = dist.cdist(np.array(objectCentroids), centroids)
            
            # in order to perform this matching we must (1) find the
            # smallest value in each row and then (2) sort the row
            # indexes based on their minimum values so that the row
            # with the smallest value is at the *front* of the index
            # list

            #[[31.90611227 38.60051813]
            # [15.5241747  14.86606875]]
            #[1 0]
            #[0 1]
            # This is an example of an output after printing D, rows and cols
            #For the rows, it will create an array with the smallest values in each row so, [31.90611227, 14.86606875].
            #In this case, 14 is smaller than 31 so the rows order will be [1 0] because 14 is in index 1.
            #For the cols, from each row, get the index of the smallest value in the same order than the rows.
            rows = D.min(axis=1).argsort()
            
            # next, we perform a similar process on the columns by
            # finding the smallest value in each column and then
            # sorting using the previously computed row index list
            cols = D.argmin(axis=1)[rows]
            
            # in order to determine if we need to update, register,
            # or deregister an object we need to keep track of which
            # of the rows and column indexes we have already examined
            usedRows = set()
            usedCols = set()

            # loop over the combination of the (row, column) index
            # tuples
            for (row, col) in zip(rows, cols):
                # if we have already examined either the row or
                # column value before, ignore it
                # val
                if row in usedRows or col in usedCols:
                    continue

                # otherwise, grab the object ID for the current row,
                # set its new centroid, and reset the disappeared
                # counter
                objectID = objectIDs[row]
                self.objects[objectID] = centroids[col]
                self.disappeared[objectID] = 0

                # indicate that we have examined each of the row and
                # column indexes, respectively
                usedRows.add(row)
                usedCols.add(col)

            # compute both the row and column index we have NOT yet
            # examined
            # substract from the total number of rows/columns the ones that are used
            unusedRows = set(range(0, D.shape[0])).difference(usedRows)
            unusedCols = set(range(0, D.shape[1])).difference(usedCols)

            # in the event that the number of object centroids is
            # equal or greater than the number of input centroids
            # we need to check and see if some of these objects have
            # potentially disappeared
            if D.shape[0] >= D.shape[1]:
                # loop over the unused row indexes
                for row in unusedRows:
                    # grab the object ID for the corresponding row
                    # index and increment the disappeared counter
                    objectID = objectIDs[row]
                    self.disappeared[objectID] += 1

                    # check to see if the number of consecutive
                    # frames the object has been marked "disappeared"
                    # for warrants deregistering the object
                    if self.disappeared[objectID] > self.maxFramesRM:
                        self.deregister(objectID)

            # otherwise, if the number of input centroids is greater
            # than the number of existing object centroids we need to
            # register each new input centroid as a trackable object
            else:
                for col in unusedCols:
                    self.register(centroids[col])

        # return the set of trackable objects
        return self.objects