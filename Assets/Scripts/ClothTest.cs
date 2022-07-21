using System;
using System.Collections.Generic;
using UnityEngine;

public class ClothTest : MonoBehaviour
{
    public ClothRenderer Cloth;
    public List<AttachPoint> AttachPoints = new List<AttachPoint>();

    private void Start()
    {
        foreach (var attachPoint in AttachPoints)
        {
            Cloth.Attach(attachPoint.VertexIndex,attachPoint.Transform);
        }
    }
}

[Serializable]
public class AttachPoint
{
    public int VertexIndex;
    public Transform Transform;
}