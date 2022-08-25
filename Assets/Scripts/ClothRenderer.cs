using System;
using System.Collections.Generic;
using DefaultNamespace;
using UnityEngine;

public class ClothRenderer : MonoBehaviour
{
    public ColliderProxy[] ColliderProxies;
    [Range(0.005f,0.02f)]
    public float DeltaTime = 0.01f;
    private MeshFilter m_MeshFilter;
    private Mesh m_Mesh;

    private ClothSimulator m_Simulator;
    private MeshModifier m_Modifier;
    private readonly Dictionary<int, Transform> m_Attachments = new();
    private float m_ElapseTime;

    private void Awake()
    {
        m_MeshFilter = GetComponent<MeshFilter>();
        m_MeshFilter.sharedMesh = Instantiate(m_MeshFilter.sharedMesh);
        m_Mesh = m_MeshFilter.sharedMesh;
        m_Modifier = MeshModifier.CreateFromMeshFilter(m_MeshFilter);
        m_Simulator = new ClothSimulator(m_Modifier);
        foreach (var proxy in ColliderProxies)
        {
            m_Simulator.AddCollider(proxy);
        }
        Debug.Log(m_Modifier.Edges.Count);
    }

    private void Start()
    {
        GetComponent<MeshRenderer>().sharedMaterial.EnableKeyword("_INPUT_WORLD_VERTEX");
        transform.localPosition = Vector3.zero;
      

    }

    /// <summary>
    /// 将指定顶点绑定到对应的坐标上
    /// </summary>
    /// <param name="vertexId"></param>
    /// <param name="target"></param>
    public void Attach(int vertexId, Transform target)
    {
        if (!target)
        {
            Debug.LogError("transform can't be null");
            return;
        }

        m_Attachments.Add(vertexId, target);
        target.position = m_Simulator.Positions[vertexId];
    }

    private void OnDestroy()
    {
        m_Simulator?.DisPose();
        GetComponent<MeshRenderer>().sharedMaterial.DisableKeyword("_INPUT_WORLD_VERTEX");
    }


    private void Update()
    {
        if (m_Simulator != null)
        {
            foreach (var keyValuePair in m_Attachments)
            {
                m_Simulator.AddPinConstraint(keyValuePair.Key, keyValuePair.Value.position);
            }
            m_ElapseTime += Time.deltaTime;
            var count = Mathf.FloorToInt(m_ElapseTime / DeltaTime);
            for (int i = 0; i < count; i++)
            {
                m_Simulator.Step(DeltaTime);
                if (i != count - 1)
                {
                    m_Simulator.FinishAllJobs();
                }
            }

            m_ElapseTime %= DeltaTime;
        }
    }

    private void LateUpdate()
    {
        if (m_Simulator!=null)
        {
            m_Simulator.FinishAllJobs();
            m_Mesh.SetVertices(m_Simulator.Positions);
            m_Mesh.SetNormals(m_Simulator.Normals);
            m_Mesh.RecalculateBounds();
        }
    }
    
    void OnDrawGizmos(){
        if(true){
            if(m_Simulator != null){
                foreach(var p in m_Simulator.Positions){
                    Gizmos.DrawSphere(p,0.05f);
                }
                for(var i = 0; i < m_Simulator.DistanceConstraintsCount;i ++){
                    var info = m_Simulator.GetDistanceConstraintInfo(i);
                    var fromPos = m_Simulator.Positions[info.VIndex0];
                    var toPos = m_Simulator.Positions[info.VIndex1];
                    Gizmos.DrawLine(fromPos,toPos);
                }
            }
        }
    }
    
}