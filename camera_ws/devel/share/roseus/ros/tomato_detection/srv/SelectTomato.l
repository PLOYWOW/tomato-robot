;; Auto-generated. Do not edit!


(when (boundp 'tomato_detection::SelectTomato)
  (if (not (find-package "TOMATO_DETECTION"))
    (make-package "TOMATO_DETECTION"))
  (shadow 'SelectTomato (find-package "TOMATO_DETECTION")))
(unless (find-package "TOMATO_DETECTION::SELECTTOMATO")
  (make-package "TOMATO_DETECTION::SELECTTOMATO"))
(unless (find-package "TOMATO_DETECTION::SELECTTOMATOREQUEST")
  (make-package "TOMATO_DETECTION::SELECTTOMATOREQUEST"))
(unless (find-package "TOMATO_DETECTION::SELECTTOMATORESPONSE")
  (make-package "TOMATO_DETECTION::SELECTTOMATORESPONSE"))

(in-package "ROS")





(defclass tomato_detection::SelectTomatoRequest
  :super ros::object
  :slots (_gripperPos ))

(defmethod tomato_detection::SelectTomatoRequest
  (:init
   (&key
    ((:gripperPos __gripperPos) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _gripperPos __gripperPos)
   self)
  (:gripperPos
   (&optional __gripperPos)
   (if __gripperPos (setq _gripperPos __gripperPos)) _gripperPos)
  (:serialization-length
   ()
   (+
    ;; float32[] _gripperPos
    (* 4    (length _gripperPos)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _gripperPos
     (write-long (length _gripperPos) s)
     (dotimes (i (length _gripperPos))
       (sys::poke (elt _gripperPos i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _gripperPos
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _gripperPos (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _gripperPos i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(defclass tomato_detection::SelectTomatoResponse
  :super ros::object
  :slots (_tomatoPos ))

(defmethod tomato_detection::SelectTomatoResponse
  (:init
   (&key
    ((:tomatoPos __tomatoPos) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _tomatoPos __tomatoPos)
   self)
  (:tomatoPos
   (&optional __tomatoPos)
   (if __tomatoPos (setq _tomatoPos __tomatoPos)) _tomatoPos)
  (:serialization-length
   ()
   (+
    ;; float32[] _tomatoPos
    (* 4    (length _tomatoPos)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _tomatoPos
     (write-long (length _tomatoPos) s)
     (dotimes (i (length _tomatoPos))
       (sys::poke (elt _tomatoPos i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _tomatoPos
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tomatoPos (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _tomatoPos i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(defclass tomato_detection::SelectTomato
  :super ros::object
  :slots ())

(setf (get tomato_detection::SelectTomato :md5sum-) "740697c9bd3592ba5c586821d8c52d19")
(setf (get tomato_detection::SelectTomato :datatype-) "tomato_detection/SelectTomato")
(setf (get tomato_detection::SelectTomato :request) tomato_detection::SelectTomatoRequest)
(setf (get tomato_detection::SelectTomato :response) tomato_detection::SelectTomatoResponse)

(defmethod tomato_detection::SelectTomatoRequest
  (:response () (instance tomato_detection::SelectTomatoResponse :init)))

(setf (get tomato_detection::SelectTomatoRequest :md5sum-) "740697c9bd3592ba5c586821d8c52d19")
(setf (get tomato_detection::SelectTomatoRequest :datatype-) "tomato_detection/SelectTomatoRequest")
(setf (get tomato_detection::SelectTomatoRequest :definition-)
      "float32[] gripperPos
---
float32[] tomatoPos

")

(setf (get tomato_detection::SelectTomatoResponse :md5sum-) "740697c9bd3592ba5c586821d8c52d19")
(setf (get tomato_detection::SelectTomatoResponse :datatype-) "tomato_detection/SelectTomatoResponse")
(setf (get tomato_detection::SelectTomatoResponse :definition-)
      "float32[] gripperPos
---
float32[] tomatoPos

")



(provide :tomato_detection/SelectTomato "740697c9bd3592ba5c586821d8c52d19")


