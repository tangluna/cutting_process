(define (stream panda-tamp)
  (:stream sample-pose
    :inputs (?o ?r)
    :domain (Stackable ?o ?r)
    :outputs (?p)
    :certified (and (Pose ?o ?p) (Supported ?o ?p ?r))
  )
  (:stream sample-grasp ; look a grasp exists
    :inputs (?a ?o)
    :domain (and (Arm ?a) (Graspable ?o)) ; this bad. should be object not property --> how make error
    ; check if obj-effect -- stream-cert is allowed in code
    ; streams output things that are always true -- add facts to state
    ; effects set what true in world; can change
    ; streams -- like look-up book. info doesnt change; only gen ONCE -- so could theoretically precomp but not in cont. world so can't do this bc too many possibilities
    ;         -- break! yay! -- extends?
    ; fact added as soon as stream added!
    ; so need to diff from grasp vs AtGrasp in potato -- piece exists from stream vs exists in world
    ; goal is derived pred?
    :outputs (?g) ; we tell what this is ground to! pddl doesn't choose! these outputs correspond to python function -- pddl only sees that grounded facts exist
    :certified (Grasp ?a ?o ?g)
  )
  (:stream generate-cut-objects ; more potatoes!!!
    :inputs (?o ?p)
    :domain (and (Cuttable ?o) (Pose ?o ?p))
    :outputs (?oh)
    :certified (and ;(Cuttable ?oh) ;(Movable ?oh1) (Graspable ?oh1) ;(Stackable ?oh1); stackable is being sus :( how add in all surfaces without python?
    ;                (Cuttable ?oh2) ;ADD OTHER RELAVENT PREDICATES
                  (CutFrom ?o ?p ?oh)
    ; add in some intuition of "CutFrom" or something that relates ?o and ?oh1
                    ) ; need to add Pose / AtPose?
  )
  (:stream inverse-kinematics
    :inputs (?a ?o ?p ?g)
    :domain (and (Arm ?a) (Pose ?o ?p) (Grasp ?a ?o ?g))
    :outputs (?q ?t)
    :certified (and (Conf ?q) (Traj ?t) (Kin ?a ?o ?p ?g ?q ?t))
  )
  (:stream plan-free-motion
    :inputs (?a ?q1 ?q2)
    :domain (and (Arm ?a) (Conf ?q1) (Conf ?q2))
    :outputs (?t)
    :certified (and (FreeMotion ?a ?q1 ?t ?q2) (Traj ?t))
  )
  (:stream plan-holding-motion
    :inputs (?a ?q1 ?q2 ?o ?g)
    :domain (and (Arm ?a) (Conf ?q1) (Conf ?q2) (Grasp ?a ?o ?g))
    :outputs (?t)
    :certified (and (HoldingMotion ?a ?q1 ?t ?q2 ?o ?g) (Traj ?t))
  )
  (:stream sample-force-grasp
    :inputs (?a ?o ?w)
    :domain (and (Arm ?a) (Graspable ?o) (Wrench ?w))
    :outputs (?g)
    :certified (and (Grasp ?a ?o ?g) (IsStableGrasp ?o ?g ?w))
  )
  (:stream plan-slice-cut-motion
    :inputs (?a ?knife ?o ?g ?p ?w1 ?w2)
    :domain (and (Arm ?a) (Knife ?knife) (Cuttable ?o) (Grasp ?a ?knife ?g) (Pose ?o ?p) (Wrench ?w1) (Wrench ?w2))
    :outputs (?q0 ?q1 ?t)
    :certified (and (SliceCutKin ?a ?knife ?o ?g ?p ?w1 ?w2 ?q0 ?q1 ?t) (Conf ?q0) (Conf ?q1) (Traj ?t))
  )
  (:stream test-pose-cfree
    :inputs (?o1 ?p1 ?o2 ?p2)
    :domain (and (Pose ?o1 ?p1) (Pose ?o2 ?p2))
    :certified (ObjCFreePose ?o1 ?p1 ?o2 ?p2)
  )
  (:stream test-traj-cfree
    :inputs (?a ?t ?o ?p)
    :domain (and (Arm ?a) (Traj ?t) (Pose ?o ?p))
    :certified (ObjCFreeTraj ?a ?t ?o ?p)
  )
  (:stream test-grasp-stable
    :inputs (?a ?o ?w ?g)
    :domain (and (Arm ?a) (Graspable ?o) (Wrench ?w) (Grasp ?a ?o ?g))
    :certified (and (IsStableGrasp ?o ?g ?w))
  )
)
