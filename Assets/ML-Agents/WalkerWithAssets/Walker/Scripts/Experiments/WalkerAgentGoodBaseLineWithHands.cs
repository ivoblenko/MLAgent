using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgentsExamples;
using Unity.MLAgents.Sensors;
using System;
public class WalkerAgentGoodBaseLineWithHands : Agent
{
    [Header("Specific to Walker")]
    [Header("Target To Walk Towards")]
    [Space(10)]
    public Transform target;

    Vector3 m_DirToTarget;
    public Transform hips;
    public Transform chest;
    public Transform spine;
    public Transform head;
    public Transform thighL;
    public Transform shinL;
    public Transform footL;
    public Transform thighR;
    public Transform shinR;
    public Transform footR;
    public Transform armL;
    public Transform forearmL;
    public Transform handL;
    public Transform armR;
    public Transform forearmR;
    public Transform handR;
    JointDriveController m_JdController;

    Rigidbody m_HipsRb;
    Rigidbody m_ChestRb;
    Rigidbody m_SpineRb;

    EnvironmentParameters m_ResetParams;

    private bool prev_foot_r = true;
    private Transform prev_footL;
    private Transform prev_footR;

    private Transform start_dot;

    private float head_height;
    private float chest_height;

    private float uppper_armR_height;
    private float start_handR_pos_y;
    private float start_handL_pos_y;

    public override void Initialize()
    {
        m_JdController = GetComponent<JointDriveController>();
        m_JdController.SetupBodyPart(hips);
        m_JdController.SetupBodyPart(chest);
        m_JdController.SetupBodyPart(spine);
        m_JdController.SetupBodyPart(head);
        m_JdController.SetupBodyPart(thighL);
        m_JdController.SetupBodyPart(shinL);
        m_JdController.SetupBodyPart(footL);
        m_JdController.SetupBodyPart(thighR);
        m_JdController.SetupBodyPart(shinR);
        m_JdController.SetupBodyPart(footR);
        m_JdController.SetupBodyPart(armL);
        m_JdController.SetupBodyPart(forearmL);
        m_JdController.SetupBodyPart(handL);
        m_JdController.SetupBodyPart(armR);
        m_JdController.SetupBodyPart(forearmR);
        m_JdController.SetupBodyPart(handR);

        m_HipsRb = hips.GetComponent<Rigidbody>();
        m_ChestRb = chest.GetComponent<Rigidbody>();
        m_SpineRb = spine.GetComponent<Rigidbody>();

        m_ResetParams = Academy.Instance.EnvironmentParameters;

        prev_footR = footR;
        prev_footL = footL;

        start_dot = hips;

        head_height = m_JdController.bodyPartsDict[head].rb.position.y - 0.1f;
        chest_height = m_JdController.bodyPartsDict[chest].rb.position.y - 0.1f;

        

        SetResetParameters();
    }

    /// <summary>
    /// Add relevant information on each body part to observations.
    /// </summary>
    public void CollectObservationBodyPart(BodyPart bp, VectorSensor sensor)
    {
        var rb = bp.rb;
        sensor.AddObservation(bp.groundContact.touchingGround ? 1 : 0); // Is this bp touching the ground
        sensor.AddObservation(rb.velocity);
        sensor.AddObservation(rb.angularVelocity);
        var localPosRelToHips = hips.InverseTransformPoint(rb.position);
        sensor.AddObservation(localPosRelToHips);

        if (bp.rb.transform != hips && bp.rb.transform != handL && bp.rb.transform != handR &&
            bp.rb.transform != footL && bp.rb.transform != footR && bp.rb.transform != head)
        {
            sensor.AddObservation(bp.currentXNormalizedRot);
            sensor.AddObservation(bp.currentYNormalizedRot);
            sensor.AddObservation(bp.currentZNormalizedRot);
            sensor.AddObservation(bp.currentStrength / m_JdController.maxJointForceLimit);
        }
    }

    /// <summary>
    /// Loop over body parts to add them to observation.
    /// </summary>
    public override void CollectObservations(VectorSensor sensor)
    {
        m_JdController.GetCurrentJointForces();

        sensor.AddObservation(m_DirToTarget.normalized);
        sensor.AddObservation(m_JdController.bodyPartsDict[hips].rb.position);
        sensor.AddObservation(hips.forward);
        sensor.AddObservation(hips.up);

        foreach (var bodyPart in m_JdController.bodyPartsDict.Values)
        {
            CollectObservationBodyPart(bodyPart, sensor);
        }
    }

    public override void OnActionReceived(float[] vectorAction)
    {
        var bpDict = m_JdController.bodyPartsDict;
        var i = -1;

        bpDict[chest].SetJointTargetRotation(vectorAction[++i], vectorAction[++i], vectorAction[++i]);
        bpDict[spine].SetJointTargetRotation(vectorAction[++i], vectorAction[++i], vectorAction[++i]);

        bpDict[thighL].SetJointTargetRotation(vectorAction[++i], vectorAction[++i], 0);
        bpDict[thighR].SetJointTargetRotation(vectorAction[++i], vectorAction[++i], 0);
        bpDict[shinL].SetJointTargetRotation(vectorAction[++i], 0, 0);
        bpDict[shinR].SetJointTargetRotation(vectorAction[++i], 0, 0);
        bpDict[footR].SetJointTargetRotation(vectorAction[++i], vectorAction[++i], vectorAction[++i]);
        bpDict[footL].SetJointTargetRotation(vectorAction[++i], vectorAction[++i], vectorAction[++i]);


        bpDict[armL].SetJointTargetRotation(vectorAction[++i], vectorAction[++i], 0);
        bpDict[armR].SetJointTargetRotation(vectorAction[++i], vectorAction[++i], 0);
        bpDict[forearmL].SetJointTargetRotation(vectorAction[++i], 0, 0);
        bpDict[forearmR].SetJointTargetRotation(vectorAction[++i], 0, 0);
        bpDict[head].SetJointTargetRotation(vectorAction[++i], vectorAction[++i], 0);

        //update joint strength settings
        bpDict[chest].SetJointStrength(vectorAction[++i]);
        bpDict[spine].SetJointStrength(vectorAction[++i]);
        bpDict[head].SetJointStrength(vectorAction[++i]);
        bpDict[thighL].SetJointStrength(vectorAction[++i]);
        bpDict[shinL].SetJointStrength(vectorAction[++i]);
        bpDict[footL].SetJointStrength(vectorAction[++i]);
        bpDict[thighR].SetJointStrength(vectorAction[++i]);
        bpDict[shinR].SetJointStrength(vectorAction[++i]);
        bpDict[footR].SetJointStrength(vectorAction[++i]);
        bpDict[armL].SetJointStrength(vectorAction[++i]);
        bpDict[forearmL].SetJointStrength(vectorAction[++i]);
        bpDict[armR].SetJointStrength(vectorAction[++i]);
        bpDict[forearmR].SetJointStrength(vectorAction[++i]);
    }

    void FixedUpdate()
    {
        // Set reward for this step according to mixture of the following elements.
        // a. Velocity alignment with goal direction.
        // b. Rotation alignment with goal direction.
        // c. Encourage head height.
        // d. Discourage head movement.
   
        //
        // // var x_chest_angle = chest.angles.x;
        // // var y_chest_angle = chest.angles.y;
        //
        // // var distance_to_target = Vector3.Distance(target.position, hips.position);
        // // AddReward(1 / distance_to_target * 100f);
        //
        // 

        // var distance_of_walk = m_JdController.bodyPartsDict[hips].rb.position.x - start_dot.position.x;
        // var distance_to_target = (1-(target.position.x - m_JdController.bodyPartsDict[hips].rb.position.x)/(target.position.x - start_dot.position.x));
        // Debug.Log(target.position.x - m_JdController.bodyPartsDict[hips].rb.position.x);
        // Debug.Log(distance_to_target);
        // AddReward(1.01f*distance_of_walk);
        //
        // AddReward(1.025f*distance_to_target);
        //
        // AddReward(0.02f * (head.position.y - hips.position.y)
        //           - 0.01f * Vector3.Distance(m_JdController.bodyPartsDict[head].rb.velocity,
        //               m_JdController.bodyPartsDict[hips].rb.velocity));

// my desicion
        m_DirToTarget = target.position - m_JdController.bodyPartsDict[hips].rb.position;
        AddReward(
            +0.03f * Vector3.Dot(m_DirToTarget.normalized, m_JdController.bodyPartsDict[hips].rb.velocity)
            + 0.01f * Vector3.Dot(m_DirToTarget.normalized, hips.forward)
            + 0.02f * (head.position.y - hips.position.y)
            - 0.01f * Vector3.Distance(m_JdController.bodyPartsDict[head].rb.velocity,
                m_JdController.bodyPartsDict[hips].rb.velocity)
        );
        var main_direction_reward =
            0.03f * Vector3.Dot(m_DirToTarget.normalized, m_JdController.bodyPartsDict[hips].rb.velocity);
        
        var forward_direction_reward = 0.01f * Vector3.Dot(m_DirToTarget.normalized, hips.forward);
        
        var yline_reward =  0.02f * (head.position.y - hips.position.y);
        
        var yline_velocity_reward = Vector3.Distance(m_JdController.bodyPartsDict[head].rb.velocity,
            m_JdController.bodyPartsDict[hips].rb.velocity);
        
        var foot_reward_type = 1f;
        
        if (m_JdController.bodyPartsDict[footR].rb.position.x > m_JdController.bodyPartsDict[footL].rb.position.x && prev_foot_r
            || m_JdController.bodyPartsDict[footR].rb.position.x < m_JdController.bodyPartsDict[footL].rb.position.x  && !prev_foot_r)
        {
            foot_reward_type = -1f;
        }
        else
        {
            prev_foot_r = !prev_foot_r;
        }
        
        var footL_step_distance = Vector3.Distance(prev_footL.localPosition, m_JdController.bodyPartsDict[footL].rb.position);
        var footR_step_distance = Vector3.Distance(prev_footR.localPosition, m_JdController.bodyPartsDict[footR].rb.position);
        var foot_step_reward = 1f;
        if (m_JdController.bodyPartsDict[footR].rb.position.x > m_JdController.bodyPartsDict[footL].rb.position.x && !prev_foot_r)
        {
            foot_step_reward = 0.01f * footR_step_distance - 0.01f * footL_step_distance;
            AddReward(0.01f*footR_step_distance);
            AddReward(-0.01f * footL_step_distance);
        } else if (m_JdController.bodyPartsDict[footR].rb.position.x < m_JdController.bodyPartsDict[footL].rb.position.x && prev_foot_r)
        {
            foot_step_reward = -0.01f * footR_step_distance + 0.01f * footL_step_distance;
            AddReward(-0.01f*footR_step_distance);
            AddReward(0.01f * footL_step_distance);
        }
        
        var footDistance = Vector3.Distance(m_JdController.bodyPartsDict[footR].rb.position, 
            m_JdController.bodyPartsDict[footL].rb.position);
        
        if (footDistance > 0.02f) footDistance = 0.02f;
        var foot_position_reward = footDistance * foot_reward_type;
        AddReward(foot_position_reward);
        
        // AddReward(-0.01f*(m_JdController.bodyPartsDict[footR].rb.position.y - 1.25f));
        // AddReward(-0.01f*(m_JdController.bodyPartsDict[footL].rb.position.y - 1.25f));
        
        var chest_rotation_y = m_JdController.bodyPartsDict[footR].rb.rotation.y;
        var chest_rotation_reward = 0.01f * (15f - chest_rotation_y);
        AddReward(chest_rotation_reward);

        float handR_reward;
        float handL_reward;

        var handR_angle = Vector3.Angle(m_JdController.bodyPartsDict[handR].rb.position,
            new Vector3(0f, 1f, 0f));
        if (handR_angle <= 15f) handR_reward = 0.02f;
        else handR_reward = -0.02f;
        
        var handL_angle = Vector3.Angle(m_JdController.bodyPartsDict[handL].rb.position,
            new Vector3(0f, 1f, 0f));
        if (handL_angle <= 15f) handL_reward = 0.02f;
        else handL_reward = -0.02f;
        
        var hand_reward = (handR_reward + handL_reward) / 2;
        AddReward(hand_reward);
        
        // var velocity_reward = ()


        // deepmind decision   
        // var standing = this.getReward(x:m_JdController.bodyPartsDict[head].rb.position.y,lower:head_height, upper:1000f,
        //     margin:(head_height / 4));
        //
        // var y_project = Vector3.Project(m_JdController.bodyPartsDict[chest].rb.position, new Vector3(0.0f, 1.0f, 0.0f)).y;
        //
        // Debug.Log(y_project);
        //
        // var upright = this.getReward(x: y_project, lower: 0.9f, upper: 1000f, sigmoid: "linear", margin: 1.9f,
        //     valua_at_margin: 0f);
        //
        // var stand_reward = standing * upright;
        //
        // var small_control = this.getReward(x:,margin:1f, value_at_margin:0f, sigmoid:"quadratic")

    }


  

    /// <summary>
    /// Loop over body parts and reset them to initial conditions.
    /// </summary>
    public override void OnEpisodeBegin()
    {
        if (m_DirToTarget != Vector3.zero)
        {
            transform.rotation = Quaternion.LookRotation(m_DirToTarget);
        }

        foreach (var bodyPart in m_JdController.bodyPartsDict.Values)
        {
            bodyPart.Reset(bodyPart);
        }
        SetResetParameters();
    }

    public void SetTorsoMass()
    {
        m_ChestRb.mass = m_ResetParams.GetWithDefault("chest_mass", 8);
        m_SpineRb.mass = m_ResetParams.GetWithDefault("spine_mass", 10);
        m_HipsRb.mass = m_ResetParams.GetWithDefault("hip_mass", 15);
    }

    public void SetResetParameters()
    {
        SetTorsoMass();
    }
    
    public float getReward(float x, float lower=0f,float upper=0f,float margin=0f,string sigmoid="guassian",float value_at_margin=0.1f)
    {
        if (lower > upper)
        {
            throw new Exception("Lower bound must be <= upper bound.");
        }

        if (margin < 0)
        {
            throw new Exception("margin must be non-negative");
        }
        
        var in_b = (lower <= x && x <= upper);
        float in_bounds;
        if (in_b)  in_bounds = 1f;
        else  in_bounds = 0f;
        float value;
        if (margin == 0)
        {
            value = in_bounds;
        }
        else
        {
            var d = x < lower ? (lower - x) / margin : (upper - x) / margin;
            value = in_bounds == 1f ? 1.0f : this._sigmoids(d, value_at_margin, sigmoid);
        }

        return value;
    }

    private float _sigmoids(float x, float value_at_1, string sigmoid)
    {
        if (sigmoid  == "cosine" || sigmoid=="linear"|| sigmoid== "quadratic"){
            if (!(0 <= value_at_1 && value_at_1 < 1))
            {
                throw new Exception("value_at_1 must be nonnegative and smaller than 1");
            }
        } else
        {
            if (!(0 < value_at_1 && value_at_1 < 1))
            {
                throw new Exception("value_at_1 must be strictly between 0 and 1");
            }
        }

        float scale;
        float scaled_x;
        if (sigmoid == "guassian")
        {
            scale = (float)Math.Sqrt(-2f * Math.Log(value_at_1));
            return (float)Math.Exp(-0.5f * (x * scale));
        } else if (sigmoid == "linear")
        {
            scale = 1 - value_at_1;
            scaled_x = x * scale;
            return (float) Math.Abs(scaled_x) < 1 ? 1 - scaled_x : 0f;
        } else if (sigmoid == "quadratic")
        {
            scale = (float)Math.Sqrt(1 - value_at_1);
            scaled_x = x * scale;
            var ret_x = Math.Abs(scaled_x) < 1 ? 1 - Math.Pow(scaled_x,2f) : 0f;
            return (float) ret_x;
        }
        else
        {
            throw new Exception("Invalid sigmoid");
        }
    }
}
