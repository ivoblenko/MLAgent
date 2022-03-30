using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgentsExamples;
using Unity.MLAgents.Sensors;
using System;
public class WalkerAgent : Agent
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

   
    private float head_height;
    private float chest_height;

    private Quaternion start_handR_rot;
    private Quaternion start_handL_rot;
    

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

     

        head_height = m_JdController.bodyPartsDict[head].rb.position.y - 0.1f;
        chest_height = m_JdController.bodyPartsDict[chest].rb.position.y - 0.1f;
        
        start_handR_rot = m_JdController.bodyPartsDict[armR].rb.rotation;
        start_handL_rot = m_JdController.bodyPartsDict[armL].rb.rotation;

        // target_walker_start_vector_xz = new Vector3(target_walker_start_vector_xz.)

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
   
     
        // my desicion
        m_DirToTarget = target.position - m_JdController.bodyPartsDict[hips].rb.position;
        AddReward(
            +0.03f * Vector3.Dot(m_DirToTarget.normalized, m_JdController.bodyPartsDict[hips].rb.velocity)
            + 0.01f * Vector3.Dot(m_DirToTarget.normalized, hips.forward)
            + 0.02f * (head.position.y - hips.position.y)
            - 0.01f * Vector3.Distance(m_JdController.bodyPartsDict[head].rb.velocity,
                m_JdController.bodyPartsDict[hips].rb.velocity)
        );
       
       
       // поощеряем за смену ног и наказываем за шаги с помощью одной ноги
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
        
        // поощеряем и наказываем за размер шага
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
        
        // контролируем расстоняние между ногами
        var footDistance = Vector3.Distance(m_JdController.bodyPartsDict[footR].rb.position, 
            m_JdController.bodyPartsDict[footL].rb.position);
        
        if (footDistance > 0.02f) footDistance = 0.02f;
        var foot_position_reward = footDistance * foot_reward_type;
        AddReward(foot_position_reward);
        
        // поощеряем и наказываем за повороты плечами
        var chest_rotation_y = m_JdController.bodyPartsDict[footR].rb.rotation.y;
        var chest_rotation_reward = 0.01f * (15f - chest_rotation_y);
        AddReward(chest_rotation_reward);

        
        // поощераяем за подконтрольное движение руками и наказываем за мотание ими
        var armR_angle = Quaternion.Angle(m_JdController.bodyPartsDict[armR].rb.rotation,
            start_handR_rot);
        
        var armL_angle = Quaternion.Angle(m_JdController.bodyPartsDict[armL].rb.rotation,
            start_handL_rot);
        
       
        var armR_reward = armR_angle < 10f ? 0.02f*(10.000001f - armR_angle) : 0.003f*(10.000001f - armR_angle); 
        var armL_reward = armL_angle < 10f ? 0.02f*(10.000001f - armL_angle) : 0.003f*(10.000001f - armL_angle); 
       
        AddReward(armR_reward + armL_reward);
       
        // поощерение за удержание кистей около таза и наказание за подьем их выше плеч
        // var handR_height_reward = 0.005f*(m_JdController.bodyPartsDict[chest].rb.position.y -
        //                    m_JdController.bodyPartsDict[handR].rb.position.y);
        // var handL_height_reward = 0.005f*(m_JdController.bodyPartsDict[chest].rb.position.y -
        //                    m_JdController.bodyPartsDict[handL].rb.position.y);
        //
        // Debug.Log("hand: "+(handR_height_reward + handL_height_reward));
      

        // AddReward(handR_height_reward + handL_height_reward);
        
        // var handR_upperhandR_angle = Vector3.Angle() 
        

        // var x_distance_reward =  Math.Abs(target.position.z - m_JdController.bodyPartsDict[hips].rb.position.z) < 10f ? 0.02f : -0.001f*Math.Abs(target.position.z - m_JdController.bodyPartsDict[hips].rb.position.z);
        // AddReward(x_distance_reward);


        
       


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

    private void hands_stabilization()
    {
        m_JdController.bodyPartsDict[armR].rb.rotation = start_handR_rot;
        m_JdController.bodyPartsDict[armL].rb.rotation = start_handL_rot;
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
