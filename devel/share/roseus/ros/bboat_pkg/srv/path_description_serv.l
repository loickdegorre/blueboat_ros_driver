;; Auto-generated. Do not edit!


(when (boundp 'bboat_pkg::path_description_serv)
  (if (not (find-package "BBOAT_PKG"))
    (make-package "BBOAT_PKG"))
  (shadow 'path_description_serv (find-package "BBOAT_PKG")))
(unless (find-package "BBOAT_PKG::PATH_DESCRIPTION_SERV")
  (make-package "BBOAT_PKG::PATH_DESCRIPTION_SERV"))
(unless (find-package "BBOAT_PKG::PATH_DESCRIPTION_SERVREQUEST")
  (make-package "BBOAT_PKG::PATH_DESCRIPTION_SERVREQUEST"))
(unless (find-package "BBOAT_PKG::PATH_DESCRIPTION_SERVRESPONSE")
  (make-package "BBOAT_PKG::PATH_DESCRIPTION_SERVRESPONSE"))

(in-package "ROS")





(defclass bboat_pkg::path_description_servRequest
  :super ros::object
  :slots ())

(defmethod bboat_pkg::path_description_servRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass bboat_pkg::path_description_servResponse
  :super ros::object
  :slots (_x _y _s _phi_f _curvature _g_c _dx _dy _ddx _ddy ))

(defmethod bboat_pkg::path_description_servResponse
  (:init
   (&key
    ((:x __x) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:y __y) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:s __s) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:phi_f __phi_f) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:curvature __curvature) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:g_c __g_c) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:dx __dx) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:dy __dy) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:ddx __ddx) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:ddy __ddy) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _x __x)
   (setq _y __y)
   (setq _s __s)
   (setq _phi_f __phi_f)
   (setq _curvature __curvature)
   (setq _g_c __g_c)
   (setq _dx __dx)
   (setq _dy __dy)
   (setq _ddx __ddx)
   (setq _ddy __ddy)
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:s
   (&optional __s)
   (if __s (setq _s __s)) _s)
  (:phi_f
   (&optional __phi_f)
   (if __phi_f (setq _phi_f __phi_f)) _phi_f)
  (:curvature
   (&optional __curvature)
   (if __curvature (setq _curvature __curvature)) _curvature)
  (:g_c
   (&optional __g_c)
   (if __g_c (setq _g_c __g_c)) _g_c)
  (:dx
   (&optional __dx)
   (if __dx (setq _dx __dx)) _dx)
  (:dy
   (&optional __dy)
   (if __dy (setq _dy __dy)) _dy)
  (:ddx
   (&optional __ddx)
   (if __ddx (setq _ddx __ddx)) _ddx)
  (:ddy
   (&optional __ddy)
   (if __ddy (setq _ddy __ddy)) _ddy)
  (:serialization-length
   ()
   (+
    ;; float64[] _x
    (* 8    (length _x)) 4
    ;; float64[] _y
    (* 8    (length _y)) 4
    ;; float64[] _s
    (* 8    (length _s)) 4
    ;; float64[] _phi_f
    (* 8    (length _phi_f)) 4
    ;; float64[] _curvature
    (* 8    (length _curvature)) 4
    ;; float64[] _g_c
    (* 8    (length _g_c)) 4
    ;; float64[] _dx
    (* 8    (length _dx)) 4
    ;; float64[] _dy
    (* 8    (length _dy)) 4
    ;; float64[] _ddx
    (* 8    (length _ddx)) 4
    ;; float64[] _ddy
    (* 8    (length _ddy)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _x
     (write-long (length _x) s)
     (dotimes (i (length _x))
       (sys::poke (elt _x i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _y
     (write-long (length _y) s)
     (dotimes (i (length _y))
       (sys::poke (elt _y i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _s
     (write-long (length _s) s)
     (dotimes (i (length _s))
       (sys::poke (elt _s i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _phi_f
     (write-long (length _phi_f) s)
     (dotimes (i (length _phi_f))
       (sys::poke (elt _phi_f i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _curvature
     (write-long (length _curvature) s)
     (dotimes (i (length _curvature))
       (sys::poke (elt _curvature i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _g_c
     (write-long (length _g_c) s)
     (dotimes (i (length _g_c))
       (sys::poke (elt _g_c i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _dx
     (write-long (length _dx) s)
     (dotimes (i (length _dx))
       (sys::poke (elt _dx i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _dy
     (write-long (length _dy) s)
     (dotimes (i (length _dy))
       (sys::poke (elt _dy i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _ddx
     (write-long (length _ddx) s)
     (dotimes (i (length _ddx))
       (sys::poke (elt _ddx i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _ddy
     (write-long (length _ddy) s)
     (dotimes (i (length _ddy))
       (sys::poke (elt _ddy i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _x
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _x (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _x i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _y
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _y (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _y i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _s
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _s (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _s i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _phi_f
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _phi_f (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _phi_f i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _curvature
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _curvature (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _curvature i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _g_c
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _g_c (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _g_c i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _dx
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _dx (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _dx i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _dy
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _dy (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _dy i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _ddx
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _ddx (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _ddx i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _ddy
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _ddy (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _ddy i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(defclass bboat_pkg::path_description_serv
  :super ros::object
  :slots ())

(setf (get bboat_pkg::path_description_serv :md5sum-) "708adc44b97d996f881dbcca00b89ef2")
(setf (get bboat_pkg::path_description_serv :datatype-) "bboat_pkg/path_description_serv")
(setf (get bboat_pkg::path_description_serv :request) bboat_pkg::path_description_servRequest)
(setf (get bboat_pkg::path_description_serv :response) bboat_pkg::path_description_servResponse)

(defmethod bboat_pkg::path_description_servRequest
  (:response () (instance bboat_pkg::path_description_servResponse :init)))

(setf (get bboat_pkg::path_description_servRequest :md5sum-) "708adc44b97d996f881dbcca00b89ef2")
(setf (get bboat_pkg::path_description_servRequest :datatype-) "bboat_pkg/path_description_servRequest")
(setf (get bboat_pkg::path_description_servRequest :definition-)
      "---
float64[] x
float64[] y
float64[] s
float64[] phi_f
float64[] curvature
float64[] g_c
float64[] dx
float64[] dy
float64[] ddx
float64[] ddy
")

(setf (get bboat_pkg::path_description_servResponse :md5sum-) "708adc44b97d996f881dbcca00b89ef2")
(setf (get bboat_pkg::path_description_servResponse :datatype-) "bboat_pkg/path_description_servResponse")
(setf (get bboat_pkg::path_description_servResponse :definition-)
      "---
float64[] x
float64[] y
float64[] s
float64[] phi_f
float64[] curvature
float64[] g_c
float64[] dx
float64[] dy
float64[] ddx
float64[] ddy
")



(provide :bboat_pkg/path_description_serv "708adc44b97d996f881dbcca00b89ef2")


