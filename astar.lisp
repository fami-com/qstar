;;; astar.el --- a fast A-star pathfinder based on minheaps  -*- lexical-binding: t; -*-

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

;;; Pathfinding obstruction declaration 

;; See the section on Pathfinding below.

(defmethod will-obstruct-p ((self node) path-finder) nil)

;;; Path class

;; finder ;; Who is finding this path?  
;; buffer ;; Pointer to associated buffer.  
;; grid ;; Array of pathfinding data pnodes.
;; height ;; Height of the pathfinding area.
;; width ;; Width of the pathfinding area.
;; heap ;; Heap array of open pathfinding pnodes.
;; end ;; Pointer to last heap array position.  
;; turn ;; Integer sequence number

(defclass path ()
  ((finder :initarg :finder :accessor path-finder)
   (buffer :initarg :buffer :accessor path-buffer)
   (grid :initarg :grid :accessor path-grid)
   (height :initarg :height :accessor path-height)
   (width :initarg :width :accessor path-width)
   (heap :initarg :heap :accessor path-heap)
   (end :initarg :end :accessor path-end) 
   (turn :initarg :turn :accessor path-turn)))

;;; Search grid resolution

(defparameter *path-grid-resolution* 180)

(defun row-to-y (path row) 
  (let ((cy (/ (slot-value (path-buffer path) 'height)
	       (path-height path))))
    (cfloat (* cy row))))

(defun column-to-x (path column) 
  (let ((cx (/ (slot-value (path-buffer path) 'width)
	       (path-width path))))
    (cfloat (* cx column))))

(defparameter *buffer-height* 1080)
(defparameter *buffer-width* 1920)

;;; Checking a grid square

(defun within-buffer-boundaries-p (buffer top left right bottom)
  (with-slots (height width) buffer
    (and (<= 0 left right width)
	 (<= 0 top bottom height))))

(defun obstructed (path row column)
  (with-slots (height width) 
      (path-buffer path)
    (let ((*quadtree* (slot-value (path-buffer path) 'quadtree))
	  (border 20))
      (multiple-value-bind (top left right bottom)
	  (bounding-box (path-finder path))
	(let* ((utop (row-to-y path row))
	       (uleft (column-to-x path column))
	       (vtop (- utop border))
	       (vleft (- uleft border))
	       (vright (+ border vleft (- right left)))
	       (vbottom (+ border vtop (- bottom top))))
	  (if 
	   (not (within-buffer-boundaries-p (current-buffer) vtop vleft vright vbottom))
	   t
	   (block colliding
	     (flet ((check (object)
		      (when (and 
			     (slot-value object 'collision-type)
			     (not (object-eq object (path-finder path)))
			     (will-obstruct-p object (path-finder path)))
			(return-from colliding object))))
	       (prog1 nil
		 (quadtree-map-collisions *quadtree*
					  (cfloat vtop)
					  (cfloat vleft)
					  (cfloat vright)
					  (cfloat vbottom)
					  #'check))))))))))

;;; Pathfinding data node 

(defstruct pnode 
  row 
  column
  parent ; previous pnode along generated path
  F ; pnode score, equal to G + H
  G ; movement cost to move from starting point
					; to (row, column) along generated path
  old-G ; previous value of G
  H ; heuristic cost to reach goal from (row, column)
  closed ; equal to path's path-turn-number when on closed list
  open ; equal to path's path-turn-number when on open list
  )

;;; Creating a pathfinding context

(defun create-path (finder &key
			     (height *path-grid-resolution*)
			     (width *path-grid-resolution*) 
			     (buffer nil))
  "Return a new path for the object FINDER in the buffer BUFFER.
  The arguments HEIGHT and WIDTH define the dimensions of the grid of
  cells to be searched for traversability."
  (assert (xelfp buffer))
  (let ((path (make-instance 'path :buffer buffer
			     :finder finder
			     :grid (make-array (list height width))
			     :heap (make-array (* height width))
			     :height height
			     :width width
			     :turn 1 :end 0)))
    (prog1 path
      (dotimes (r height)
	(dotimes (c width)
	  (setf (aref (path-grid path) r c)
		(make-pnode :row r :column c)))))))

;;; Minheap for open and closed node sets

(defun open-pnode (path pnode)
  (let* ((path-heap-end (if (null (path-end path))
			    (setf (path-end path) 1)
			    (incf (path-end path))))
	 (path-heap (path-heap path))
	 (ptr path-heap-end)
	 (parent nil)
	 (finished nil))
    ;; make it easy to check whether pnode is open
    (setf (pnode-open pnode) (path-turn path))
    ;; add pnode to end of heap 
    (setf (aref path-heap path-heap-end) pnode)
    ;; let pnode rise to appropriate place in heap
    (while (and (not finished) (< 1 ptr))
      (setf parent (truncate (/ ptr 2)))
      ;; should it rise? 
      (if (< (pnode-F pnode) (pnode-F (aref path-heap parent)))
	  ;; yes. swap parent and pnode
	  (progn 
	    (setf (aref path-heap ptr) (aref path-heap parent))
	    (setf ptr parent))
	  ;; no. we're done.
	  (progn (setf finished t)
		 (setf (aref path-heap ptr) pnode))))
    ;; do we need to set pnode as the new root? 
    (if (and (not finished) (equal 1 ptr))
	(setf (aref path-heap 1) pnode))))

(defun close-pnode (path)
  (let* ((path-heap (path-heap path))
	 ;; save root of heap to return to caller
	 (pnode (aref path-heap 1))
	 (last nil)
	 (path-heap-end (path-end path))
	 (ptr 1)
	 (left 2)
	 (right 3)
	 (finished nil))
    ;; is there only one pnode?
    (if (equal 1 path-heap-end)
	(setf (path-end path) nil)
	(if (null path-heap-end)
	    nil
	    ;; remove last pnode of heap and install as root of heap
	    (progn
	      (setf last (aref path-heap path-heap-end))
	      (setf (aref path-heap 1) last)
	      ;; shrink heap
	      (decf (path-end path))
	      (decf path-heap-end)
	      ;;
	      (setf (pnode-closed pnode) (path-turn path))
	      ;;
	      ;; figure out where former last element should go
	      ;;
	      (while (and (not finished) (>= path-heap-end right))
		;;
		;; does it need to sink? 
		(if (and (< (pnode-F last) (pnode-F (aref path-heap left)))
			 (< (pnode-F last) (pnode-F (aref path-heap right))))
		    ;;
		    ;; no. we're done
		    (progn 
		      (setf finished t)
		      (setf (aref path-heap ptr) last))
		    ;;
		    ;; does it need to sink rightward?
		    (if (>= (pnode-F (aref path-heap left)) 
			    (pnode-F (aref path-heap right)))
			;;
			;; yes
			(progn
			  (setf (aref path-heap ptr) (aref path-heap right))
			  (setf ptr right))
			;;
			;; no, sink leftward
			(progn
			  (setf (aref path-heap ptr) (aref path-heap left))
			  (setf ptr left))))
		(setf left (* 2 ptr))
		(setf right (+ 1 left)))
	      ;;
	      ;; 
	      (if (and (equal left path-heap-end)
		       (> (pnode-F last)
			  (pnode-F (aref path-heap left))))
		  (setf ptr left)))))
    ;;
    ;; save former last element in its new place
    (setf (aref path-heap ptr) last)
    pnode))

;;; Scoring pathfinding nodes

(defun score-pnode (path pnode path-turn-number new-parent-pnode
		    goal-row goal-column)
  "Update scores for PNODE. Update heap position if necessary."
  (let* ((direction (find-direction (pnode-column new-parent-pnode)
				    (pnode-row new-parent-pnode)
				    (pnode-column pnode)
				    (pnode-row pnode)))
	 (G (+ 1 (pnode-G new-parent-pnode)))
	 
	 (H (* (distance (pnode-column pnode)
			 (pnode-row pnode)
			 goal-column goal-row)
	       ;; (max (abs (- (pnode-row pnode) goal-row))
	       ;;     (abs (- (pnode-column pnode) goal-column)))
	       1))
	 (F (+ G H)))
    ;; 
    ;; is this a new pnode, i.e. not on the open list? 
    (if (not (equal path-turn-number (pnode-open pnode)))
	;;
	;; yes, update its scores and parent
	(progn 
	  (setf (pnode-G pnode) G)
	  (setf (pnode-H pnode) H)
	  (setf (pnode-F pnode) F)
	  (setf (pnode-parent pnode) new-parent-pnode))
	;;
	;; no, it's already open. is the path through NEW-PARENT-PNODE
	;; better than through the old parent?
	(if (and (pnode-G pnode)
		 (< G (pnode-G pnode)))
	    ;;
	    ;; yes. update scores and re-heap.
	    (let ((heap (path-heap path))
		  (heap-end (path-end path))
		  (ptr 1)
		  (par nil)
		  (finished nil))
	      (setf (pnode-G pnode) G)
	      (setf (pnode-H pnode) H)
	      (setf (pnode-F pnode) F)
	      (setf (pnode-parent pnode) new-parent-pnode)
	      ;;
	      ;; Better score found.
	      ;; 
	      ;; find current location of pnode in heap
	      (while (and (not finished) (< ptr heap-end))
		(when (equal pnode (aref heap ptr))
		  ;; Found pnode.
		  ;;
		  ;; its score could only go down, so move it up in the
		  ;; heap if necessary.
		  (while (and (not finished) (< 1 ptr))
		    (setf par (truncate (/ ptr 2)))
		    ;;
		    ;; should it rise? 
		    (if (< (pnode-F pnode) (pnode-F (aref heap par)))
			;;
			;; yes. swap it with its parent
			(progn
			  (setf (aref heap ptr) (aref heap par))
			  (setf ptr par))
			;;
			;; no, we are done. put pnode in its new place.
			(progn (setf finished t)
			       (setf (aref heap ptr) pnode))))
		  ;;
		  ;; do we need to install the new pnode as heap root?
		  (when (and (not finished) (equal 1 ptr))
		    (setf (aref heap 1) pnode)))
		;;
		;; keep scanning heap for the pnode
		(incf ptr)))
	    ;;
	    ;; new score is not better. do nothing.
					;(setf (pnode-parent pnode) new-parent-pnode)
	    ))))

;;; Finding successor nodes in search

(defun pnode-successors (path pnode path-turn-number goal-row
			 goal-column)
  (delete nil 
	  (mapcar 
	   #'(lambda (direction)
	       (let ((grid (path-grid path))
		     (new-G (+ 1 (pnode-G pnode)))
		     (successor nil))
		 (multiple-value-bind (r c) 
		     (step-in-direction 
		      (pnode-row pnode)
		      (pnode-column pnode)
		      direction)
		   ;; 
		   (if (array-in-bounds-p grid r c)
		       (progn 
			 (setf successor (aref grid r c))
			 
			 (if (or 
			      ;; always allow the goal square even when it's an obstacle.
			      (and (equal r goal-row) (equal c goal-column))
			      ;; ignore non-walkable squares and closed squares,
			      (and (not (obstructed path r c))
				   (not (equal path-turn-number (pnode-closed successor)))))
			     ;; if successor is open and existing path is better
			     ;; or as good as new path, destroy the successor
			     ;; if successor is not open, proceed 
			     (if (equal path-turn-number (pnode-open successor))
				 (if (< new-G (pnode-G successor))
				     successor
				     nil)
				 successor)
			     nil))
		       nil))))
	   *directions*)))

;;; Core pathfinding routine

(defun address-to-waypoint (path address)
  (destructuring-bind (row column) address
    (list (round (column-to-x path column))
	  (round (row-to-y path row)))))

(defun find-path (path x0 y0 x1 y1)
  (let* ((selected-pnode nil)
	 (path-turn-number (incf (path-turn path)))
	 (pos nil)
	 (found nil)
	 (path-height (path-height path))
	 (path-width (path-width path))
	 (buffer-height *buffer-height*) 
	 (buffer-width *buffer-width*)
	 (cx (/ buffer-width path-width))
	 (cy (/ buffer-height path-height))
	 (target-pnode nil)
	 (coordinates nil)
	 (F 0) (G 0) (H 0)
	 (starting-row (round (/ y0 cy)))
	 (starting-column (round (/ x0 cx)))
	 (goal-row (round (/ y1 cy)))
	 (goal-column (round (/ x1 cx))))
    (if (obstructed path goal-row goal-column)
	(prog1 nil) ;; (message "Not pathfinding to obstructed area.")
	(progn 
	  ;; reset the pathfinding heap
	  (setf (path-end path) nil)
	  ;; add the starting pnode to the open set
	  (setf G 0)
	  (setf H (max (abs (- starting-row goal-row))
		       (abs (- starting-column goal-column))))
	  (setf F (+ G H))
	  (setf selected-pnode (make-pnode :row starting-row 
					   :column starting-column
					   :old-G 0
					   :parent nil :G G :F F :H H))
	  ;;
	  (open-pnode path selected-pnode)
	  ;; start pathfinding
	  (setf target-pnode
		(block finding
		  ;; select and close the pnode with smallest F score
		  (while (setf selected-pnode (close-pnode path))
		    ;; did we fail to reach the goal? 
		    (when (null selected-pnode)
		      (return-from finding nil))
		    ;; are we at the goal square?
		    (when (and (equal goal-row (pnode-row selected-pnode))
			       (equal goal-column (pnode-column selected-pnode)))
		      (return-from finding selected-pnode))
		    ;; process adjacent walkable non-closed pnodes
		    (mapc #'(lambda (pnode)
			      ;; is this cell already on the open list?
			      (if (equal path-turn-number (pnode-open pnode))
				  ;; yes. update scores if needed
				  (score-pnode path pnode path-turn-number
					       selected-pnode goal-row goal-column)
				  (progn 
				    ;; it's not on the open list. add it to the open list
				    (score-pnode path pnode path-turn-number selected-pnode
						 goal-row goal-column)
				    (open-pnode path pnode))))
			  ;; map over adjacent pnodes
			  (pnode-successors path selected-pnode 
					    path-turn-number
					    goal-row goal-column)))))
	  ;; did we find a path? 
	  (if (pnode-p target-pnode)
	      ;; save the path by walking backwards from the target
	      (let ((previous-pnode target-pnode)
		    (current-pnode nil)
		    (dirs nil))
		(while (setf current-pnode (pnode-parent previous-pnode))
		  ;; what direction do we travel to get from current to previous? 
		  (push (list (pnode-row current-pnode)
			      (pnode-column current-pnode))
			coordinates)
		  (push (find-direction
			 (pnode-column current-pnode)
			 (pnode-row current-pnode)
			 (pnode-column previous-pnode)
			 (pnode-row previous-pnode))
			dirs)
		  (setf previous-pnode current-pnode))
		;; return the finished path
		(values coordinates dirs))
	      ;; return nil
	      nil)))))

(defun find-path-waypoints (path x0 y0 x1 y1)
  "Find a path from the starting point to the goal in PATH using A*.
  Returns a list of coordinate waypoints an AI can follow to reach the
  goal."
  (mapcar #'(lambda (address)
	      (address-to-waypoint path address))
	  (find-path path (truncate x0) 
		     (truncate y0)
		     (truncate x1)
		     (truncate y1))))

(defgeneric collide (this that)
  (:documentation
   "Trigger defined collision methods for when THIS collides with THAT.
    If a collision method is defined as (COLLIDE CLASS-1 CLASS-2), then
    this COLLIDE will trigger when QUADTREE-COLLIDE is called on instances
    of CLASS-1 that are colliding with instances of CLASS-2. 

    If (COLLIDE CLASS-2 CLASS-1) is also defined, it will be triggered
    only when QUADTREE-COLLIDE is called on colliding instances of
    CLASS-2.

    If you always want both orderings of the class pair's COLLIDE to be
    triggered, then you must call QUADTREE-COLLIDE on every object in the
    scene. This is done by default but can be interfered with if you use
    the slot COLLISION-TYPE."))

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

;;; astar.lisp ends here
