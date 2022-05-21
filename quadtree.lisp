;;; quadtree.lisp --- fast collision detection for axis-aligned bounding boxes

;;; Copyright (C) 2008-2018  David O'Toole

;; Author: David O'Toole <dto@xelf.me>

;; Permission is hereby granted, free of charge, to any person obtaining a copy
;; of this software and associated documentation files (the "Software"), to deal
;; in the Software without restriction, including without limitation the rights
;; to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
;; copies of the Software, and to permit persons to whom the Software is
;; furnished to do so, subject to the following conditions:

;; The above copyright notice and this permission notice shall be included in all
;; copies or substantial portions of the Software.

;; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
;; IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
;; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
;; AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
;; LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
;; OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
;; SOFTWARE.

(in-package :qstar)

;;; Emacs Lisp compatibility  

;; The WHILE syntax was used in the original Emacs Lisp version of
;; this package.

(defmacro while (test &body body)
  `(loop while ,test do ,@body))

(defun within-extents (x y x0 y0 x1 y1)
  "Return non-nil when point X,Y is within rectangle X0,Y0,X1,Y1."
  (and (>= x x0) 
       (<= x x1)
       (>= y y0)
       (<= y y1)))

;;; Bounding box data

;; Below you can see that bounding boxes are always given in the order
;; TOP, LEFT, RIGHT, BOTTOM, whether provided in a list or as multiple
;; return values. 

(defun cfloat (f)
  (coerce f 'single-float))

(defun valid-bounding-box-p (box)
  "Return non-nil if BOX is a spatially valid bounding box.
   Bounding boxes are lists of the form (TOP LEFT RIGHT BOTTOM)."
  (and (listp box)
       (= 4 (length box))
       (destructuring-bind (top left right bottom) box
	 (and (<= left right) (<= top bottom)))))

(defun bounding-box-contains (box0 box1)
  "Test whether BOX0 contains BOX1. The bounding boxes are provided as
   lists of the form (TOP LEFT RIGHT BOTTOM)."
  (destructuring-bind (top0 left0 right0 bottom0) box0
    (destructuring-bind (top1 left1 right1 bottom1) box1
      (declare (single-float top0 left0 right0 bottom0 top1 left1 right1 bottom1) 
	       (optimize (speed 3)))
      (and (<= top0 top1)
	   (<= left0 left1)
	   (>= right0 right1)
	   (>= bottom0 bottom1)))))

;;; The active quadtree

(defvar *quadtree* nil
  "When non-nil, the current quadtree object. See WITH-QUADTREE")

(defmacro with-quadtree (quadtree &body body)
  "Evaluate BODY forms with *QUADTREE* bound to QUADTREE."
  `(let* ((*quadtree* ,quadtree))
     ,@body))

(defvar *quadtree-depth* 0 "Current depth of the quadtree.")
(defparameter *default-quadtree-depth* 6 "Default quadtree depth.")

;;; Data structure

;; In each quadtree node we need a bucket of objects, an integer tree
;; level ID, a bounding box, and downward links to the child quadtree
;; nodes.

(defclass quadtree ()
  ((objects :initform nil :accessor quadtree-objects :initarg :objects)
   (level :initform nil :accessor quadtree-level :initarg :level)
   (top :initform nil :accessor quadtree-top :initarg :top)
   (left :initform nil :accessor quadtree-left :initarg :left)
   (right :initform nil :accessor quadtree-right :initarg :right)
   (bottom :initform nil :accessor quadtree-bottom :initarg :bottom)
   (southwest :initform nil :accessor quadtree-southwest :initarg :southwest)
   (northeast :initform nil :accessor quadtree-northeast :initarg :northeast)
   (northwest :initform nil :accessor quadtree-northwest :initarg :northwest)
   (southeast :initform nil :accessor quadtree-southeast :initarg :southeast)))

;;; Quadrille: base class for collidable objects

;; A QUADRILLE is an object which maintains a constant relationship to
;; the currently active quadtree.

(defclass quadrille ()
  ((quadtree-node :initform nil :initarg :quadtree-node :accessor quadtree-node)
   (collision-type :initform t :initarg :collision-type :accessor collision-type)
   (heading :initform 0.0 :accessor heading)
   (width :initform 32 :accessor width)
   (height :initform 32 :accessor height)
   (x :initform (cfloat 0) :accessor x)
   (y :initform (cfloat 0) :accessor y)
   (z :initform (cfloat 0) :accessor z)
   (last-x :initform nil :accessor last-x)
   (last-y :initform nil :accessor last-y)
   (last-z :initform nil :accessor last-z)))

;;; Computing subtree coordinates

;; Each quadtree node's space is subdivided equally into four
;; quadrants. These recursively smaller bounding boxes define the spatial
;; partitioning of the quadtree.

(defun northeast-quadrant (bounding-box)
  (destructuring-bind (top left right bottom) bounding-box
    (list top (cfloat (/ (+ left right) 2)) right (cfloat (/ (+ top
								bottom) 2)))))

(defun southeast-quadrant (bounding-box)
  (destructuring-bind (top left right bottom) bounding-box
    (list (cfloat (/ (+ top bottom) 2)) (cfloat (/ (+ left right) 2))
	  right bottom)))

(defun northwest-quadrant (bounding-box)
  (destructuring-bind (top left right bottom) bounding-box
    (list top left
	  (cfloat (/ (+ left right) 2)) (cfloat (/ (+ top bottom) 2)))))

(defun southwest-quadrant (bounding-box)
  (destructuring-bind (top left right bottom) bounding-box
    (list (cfloat (/ (+ top bottom) 2)) left
	  (cfloat (/ (+ left right) 2)) bottom)))

;;; Building a quadtree structure recursively

(defun build-quadtree (bounding-box0 &optional (depth
						*default-quadtree-depth*))
  "Build a complete quadtree structure inside BOUNDING-BOX0 with DEPTH levels."
  (let ((bounding-box (mapcar #'cfloat bounding-box0)))
    (destructuring-bind (top left right bottom) bounding-box
      (decf depth)
      (if (zerop depth)
	  (make-instance 'quadtree :top top :left left :right right :bottom bottom)
	  (make-instance 'quadtree :top top :left left :right right :bottom bottom
			 :northwest (build-quadtree (northwest-quadrant bounding-box) depth)
			 :northeast (build-quadtree (northeast-quadrant bounding-box) depth)
			 :southwest (build-quadtree (southwest-quadrant bounding-box) depth)
			 :southeast (build-quadtree (southeast-quadrant bounding-box) depth))))))

;;; User-level MAKE-QUADTREE function

;; Building quadtrees of depths between 5 and 8 works well for most
;; games; depths larger than 10 may be more efficient for large-sized
;; buffers and/or when many small objects are being simulated, but such
;; quadtrees will take much more memory. See also [[file:dictionary:_DEFAULT-QUADTREE-DEPTH_.html][*DEFAULT-QUADTREE-DEPTH*]].

(defun make-quadtree (x y width height 
		      &key objects (depth *default-quadtree-depth*))
  "Make a new quadtree with the given dimensions, OBJECTS, and DEPTH."
  (let ((quadtree (build-quadtree (list y x (+ x width) (+ y height)) depth)))
    (when objects
      (quadtree-fill objects quadtree))
    quadtree))

;;; Testing whether a quadtree node is a leaf node

(defmethod leafp ((node quadtree))
  "Return non-nil if NODE has no children."
  ;; this is a complete tree, so testing any quadrant will suffice
  (null (quadtree-southwest node)))

;;; Testing quadtree nodes against points and rectangles 

(defmethod quadtree-contains ((quadtree quadtree) top left right
			      bottom)
  "Return non-nil if the node QUADTREE contains the given bounding box."
  (declare (single-float top left right bottom) (optimize (speed 3)))
  (and (<= (the single-float (quadtree-top quadtree)) top)
       (<= (the single-float (quadtree-left quadtree)) left)
       (>= (the single-float (quadtree-right quadtree)) right)
       (>= (the single-float (quadtree-bottom quadtree)) bottom)))

;;; Traversing the quadtree

(defmethod quadtree-process ((node quadtree) top left right bottom
			     processor)
  "Call the function PROCESSOR on each quadtree node containing the bounding box."
  (when (quadtree-contains node top left right bottom)
    (when (not (leafp node))
      (quadtree-process (quadtree-northwest node) top left right bottom processor)
      (quadtree-process (quadtree-northeast node) top left right bottom processor)
      (quadtree-process (quadtree-southwest node) top left right bottom processor)
      (quadtree-process (quadtree-southeast node) top left right bottom processor))
    (funcall processor node)))

;;; Bounding-box search

;; QUADTREE-SEARCH is the hashing function in our spatial hash; the
;; bounding box is the key and the value is the correct bucket
;; (i.e. quadtree node).

(defun quadtree-search (top left right bottom node)
  "Return the smallest quadrant enclosing TOP LEFT RIGHT BOTTOM at or
  below NODE, if any."
  (when (quadtree-contains node top left right bottom)
    ;; ok, it's in the overall bounding-box.
    (if (leafp node)
	;; there aren't any quadrants to search. stop here.
	node
	(or
	 ;; search the quadrants.
	 (or (quadtree-search top left right bottom (quadtree-northwest node))
	     (quadtree-search top left right bottom (quadtree-northeast node))
	     (quadtree-search top left right bottom (quadtree-southwest node))
	     (quadtree-search top left right bottom (quadtree-southeast node)))
	 ;; none of them are suitable. stay here
	 node))))

;;; Inserting objects

(defgeneric quadtree-insert (object tree)
  (:documentation
   "Insert the object OBJECT into the quadtree TREE."))

(defmethod quadtree-insert ((object quadrille) (tree quadtree))
  (let ((node0 
	 (multiple-value-bind (top left right bottom) (bounding-box object)
	   (quadtree-search top left right bottom tree))))
    (let ((node (or node0 tree)))
      (pushnew (find-identifier object)
	       (quadtree-objects node)
	       :test 'eq)
      ;; save pointer to node so we can avoid searching when it's time
      ;; to delete (i.e. move) the object later.
      (setf (quadtree-node object) node))))

(defmethod quadtree-insert-maybe ((object quadrille) tree)
  (when tree
    (quadtree-insert object tree)))

;;; Deleting objects

(defgeneric quadtree-delete (object tree)
  (:documentation
   "Delete the object OBJECT from the quadtree TREE."))

(defmethod quadtree-delete ((object quadrille) (tree quadtree))
  ;; grab the cached quadtree node
  (let ((node (or (quadtree-node object) tree)))
    (setf (quadtree-objects node)
	  (delete (find-identifier object) (quadtree-objects node) :test 'eq))
    (setf (quadtree-node object) nil)))

(defmethod quadtree-delete-maybe ((object quadrille) tree)
  (when (and tree (quadtree-node object))
    (quadtree-delete object tree)))

;;; Moving objects

;; The method UPDATE-BOUNDING-BOX is invoked whenever an object's
;; bounding box changes. Since this may change its quadtree cell, we
;; remove it from the tree and re-insert it to its proper location
;; after changing the bounding box. 

(defgeneric update-bounding-box (object quadtree)
  (:documentation 
   "Update the OBJECT's new bounding box and position in QUADTREE."))

(defmethod update-bounding-box ((object quadrille) tree)
  (with-quadtree tree
    (quadtree-delete-maybe object tree)
    (quadtree-insert-maybe object tree)))

;;; Inserting many objects into a quadtree

(defun quadtree-fill (set quadtree)
  "Insert the objects in SET (a list, vector or a hashtable) into QUADTREE."
  (flet ((insert (object)
           (setf (quadtree-node object) nil)
           (quadtree-insert object quadtree)))
    (etypecase set
      (list (loop :for object :in set :do (insert object)))
      (vector (loop :for object :across set :do (insert object)))
      (hash-table (loop :for object :being :the :hash-keys :in set :do (insert object))))))

;;; Collision geometry tests

(defun rectangle-in-rectangle-p (x y width height o-top o-left o-width
				 o-height)
  (declare (single-float x y width height o-top o-left o-width o-height)
	   (optimize (speed 3)))
  (not (or 
	;; is the top below the other bottom?
	(<= (+ o-top o-height) y)
	;; is bottom above other top?
	(<= (+ y height) o-top)
	;; is right to left of other left?
	(<= (+ x width) o-left)
	;; is left to right of other right?
	(<= (+ o-left o-width) x))))

(defmethod colliding-with-rectangle-p ((self quadrille) o-top o-left o-width o-height)
  ;; you must pass arguments in Y X order since this is TOP then LEFT
  (multiple-value-bind (x y width height) (bounding-box* self)
    (rectangle-in-rectangle-p (cfloat x) (cfloat y) (cfloat width) (cfloat height) 
			      (cfloat o-top) (cfloat o-left) (cfloat o-width) (cfloat o-height))))

(defun colliding-with-bounding-box-p (self top left right bottom)
  ;; you must pass arguments in Y X order since this is TOP then LEFT
  (multiple-value-bind (x y width height) (bounding-box* self)
    (when (and width height)
      (rectangle-in-rectangle-p (cfloat x) (cfloat y) (cfloat width) (cfloat height)
				top left (- right left) (- bottom top)))))

(defgeneric colliding-with-p (this that)
  (:documentation 
   "Return non-nil when bounding boxes of THIS and THAT are colliding."))

(defmethod colliding-with-p ((self quadrille) (thing quadrille))
  (multiple-value-bind (top left right bottom) 
      (bounding-box thing)
    (colliding-with-bounding-box-p self top left right bottom)))

;;; Geometry utilities

(defgeneric bounding-box (object)
  (:documentation 
   "Return the bounding-box of this OBJECT as multiple values.
  The proper VALUES ordering is (TOP LEFT RIGHT BOTTOM), which could
  also be written (Y X (+ X WIDTH) (+ Y HEIGHT)) if more convenient."))

(defmethod bounding-box ((quadrille quadrille))
  "Return this object's bounding box as multiple values.
  The order is (TOP LEFT RIGHT BOTTOM)."
  (with-slots (x y width height) quadrille
    (values 
     (cfloat y)
     (cfloat x)
     (cfloat (+ x width))
     (cfloat (+ y height)))))

(defmethod bounding-box* ((quadrille quadrille))
  (multiple-value-bind (top left right bottom) (bounding-box quadrille)
    (values left top (- right left) (- bottom top))))

(defmethod center-point ((quadrille quadrille))
  (multiple-value-bind (top left right bottom)
      (the (values float float float float) (bounding-box quadrille))
    (let ((half (cfloat 0.5)))
      (declare (single-float half top left right bottom) (optimize (speed 3)))
      (values (* half (+ left right))
	      (* half (+ top bottom))))))

(defmethod heading-to-thing2 ((self quadrille) thing)
  (multiple-value-bind (x1 y1) (center-point thing)
    (multiple-value-bind (x0 y0) (center-point self)
      (find-heading x0 y0 x1 y1))))

(defmethod heading-to-thing ((self quadrille) thing)
  (with-slots (x y) self 
    (multiple-value-bind (x0 y0) (center-point thing)
      (find-heading x y x0 y0))))

(defmethod heading-between ((self quadrille) thing)
  (multiple-value-bind (x y) (center-point self)
    (multiple-value-bind (x0 y0) (center-point thing)
      (find-heading x y x0 y0))))

(defmethod aim-at  ((self quadrille) node)
  (setf (heading self) (heading-between self node)))

(defmethod aim  ((self quadrille) heading)
  (assert (numberp heading))
  (setf (heading self) heading))

(defmethod distance-between  ((self quadrille) (thing quadrille))
  (multiple-value-bind (x0 y0) (center-point self)
    (multiple-value-bind (x y) (center-point thing)
      (distance x0 y0 x y))))

;;; Movement

;; Because an object's bounding box determines its position in the
;; quadtree, we must delete and then re-insert that object whenever its
;; size or position changes. See also "Quadtrees" above.

(defmethod move-to ((node quadrille) x0 y0 &optional z0)
  (with-slots (x y z) node
    (setf x x0 y y0)
    (when z (setf z z0)))
  (update-bounding-box node *quadtree*)
  nil)

(defmethod resize ((quadrille quadrille) width height)
  (quadtree-delete-maybe quadrille (quadtree-node quadrille))
  (setf (height quadrille) height)
  (setf (width quadrille) width)
  (quadtree-insert-maybe quadrille (quadtree-node quadrille)) nil)

;;; All the other movement and resize functions are based on MOVE-TO and
;; RESIZE, so the quadtree data are maintained properly.

(defun step-coordinates (x y heading &optional (distance 1))
  "Return as multiple values the coordinates of the point DISTANCE
    units away from X,Y in the direction given by HEADING."
  (values (+ x (* distance (cos heading)))
	  (+ y (* distance (sin heading)))))

(defmethod step-toward-heading ((self quadrille) heading &optional (distance 1))
  (multiple-value-bind (x y) (center-point self)
    (step-coordinates x y heading distance)))

(defmethod move  ((self quadrille) heading distance)
  (with-slots (x y) self
    (multiple-value-bind (x0 y0) (step-coordinates x y heading distance)
      (move-to self x0 y0))))

(defmethod forward  ((self quadrille) distance)
  (move self (heading self) distance))

(defmethod backward  ((self quadrille) distance)
  (move self (opposite-heading (heading self)) distance))

(defmethod move-toward ((self quadrille) direction &optional (steps 1))
  (with-slots (x y) self
    (multiple-value-bind (x0 y0)
	(step-in-direction x y (or direction :up) (or steps 5))
      (move-to self x0 y0))))

(defmethod turn-left ((self quadrille) radians)
  (decf (heading self) radians))

(defmethod turn-right ((self quadrille) radians)
  (incf (heading self) radians))

;;; Tracking pre-collision locations

;; This makes it easy to undo movements that led to collisions.

(defmethod save-location ((quadrille quadrille))
  (with-slots (x y z last-x last-y last-z) quadrille
    (setf last-x x
	  last-y y
	  last-z z)))

(defmethod clear-saved-location ((quadrille quadrille))
  (with-slots (last-x last-y last-z) quadrille
    (setf last-x nil last-y nil last-z nil)))

(defmethod restore-location ((quadrille quadrille))
  (with-slots (x y z last-x last-y last-z quadtree-node) quadrille
    (when last-x
      (quadtree-delete-maybe quadrille quadtree-node)
      (setf x last-x
	    y last-y
	    z last-z)
      (quadtree-insert-maybe quadrille quadtree-node))))

;;; Responding to collisions

;; The main method for an object's collision response is [[file:dictionary/COLLIDE.html][COLLIDE]].

(defgeneric collide (this that)
  (:documentation
   "Trigger defined collision methods for when THIS collides with THAT.
    If a collision method is defined as (COLLIDE CLASS-1 CLASS-2), then
    this COLLIDE will trigger when QUADTREE-COLLIDE is called on instances
    of CLASS-1 that are colliding with instances of CLASS-2. 

    If (COLLIDE CLASS-2 CLASS-1) is also defined, it will be triggered
    only when QUADTREE-COLLIDE is called on colliding instances of
    CLASS-2. 

    If you always want both orderings of the class pair's COLLIDE to
    be triggered, then you must call QUADTREE-COLLIDE on every object
    in the tree. This is done by default, but can be interfered with
    if you use the slot COLLISION-TYPE.

    Whether or not (COLLIDE CLASS-1 CLASS-2) happens before or after (COLLIDE
    CLASS-2 CLASS-1) is undefined.
    "))

(defmethod collide ((this quadrille) (that quadrille)) nil)

(defmethod handle-collision ((this quadrille) (that quadrille)) 
  (collide this that)
  (collide that this))

;;; Bounding box query

;; We need a method to check a quadtree bucket against a given bounding
;; box and process all the colliding objects.

(defmethod quadtree-map-collisions ((tree quadtree) top left right
				    bottom processor)
  (quadtree-process tree top left right bottom
		    #'(lambda (node)
			(let (garbage)
			  (dolist (object (quadtree-objects node))
			    (if (search-identifier object)
				(when (colliding-with-bounding-box-p (search-identifier object) top left right bottom)
				  (funcall processor (search-identifier object)))
				(push object garbage)))
			  (dolist (g garbage)
			    (setf (quadtree-objects node)
				  (delete g (quadtree-objects node) :test 'equal)))))))

;;; Top-level collision triggers

(defgeneric quadtree-collide (object quadtree)
  (:documentation
   "Detect and handle collisions of OBJECT with other objects within the
  QUADTREE. The multimethod COLLIDE will be invoked on each pair of 
  (OBJECT OTHER-OBJECT)"))

(defmethod quadtree-collide ((object quadrille) (tree quadtree))
  (multiple-value-bind (top left right bottom) (bounding-box object)
    (quadtree-map-collisions tree
			     top left right bottom
			     #'(lambda (thing)
				 (when (and (collision-type thing)
					    (colliding-with-p object thing)
					    (not (eq object thing)))
				   (with-quadtree tree
				     (handle-collision object thing)))))))

(defun collide-objects (objects quadtree)
  "Trigger all collisions for OBJECTS within QUADTREE."
  (dolist (object objects)
    (quadtree-collide object quadtree)))

(defun collide-objects* (objects quadtree)
  "Trigger all collisions for identified OBJECTS within QUADTREE."
  (dolist (id (mapcar #'find-identifier objects))
    ;; object might have been deleted by other collisions
    (when (search-identifier id)
      (quadtree-collide (search-identifier id) quadtree))))

;;; quadtree.lisp ends here
