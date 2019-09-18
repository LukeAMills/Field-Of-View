using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class FieldOfView : MonoBehaviour {

	public float viewRadius;
	[Range(0,360)]
	public float viewAngle;

	public LayerMask targetMask;
	public LayerMask obstacleMask;

	[HideInInspector]
	public List<Transform> visibleTargets = new List<Transform>();

    public float meshResolution;    //settable variable for resolution, determines how many rays get cast out
    public MeshFilter viewMeshFilter;
    public int edgeResolveIterations;
    public float edgeDistanceThreshold;
    public float maskCutawayDst = .1f;
    Mesh viewMesh;

	void Start()
    {
        viewMesh = new Mesh();
        viewMesh.name = "View Mesh";
        viewMeshFilter.mesh = viewMesh;
		StartCoroutine ("FindTargetsWithDelay", .2f);
	}


	IEnumerator FindTargetsWithDelay(float delay) {
		while (true)
        {
			yield return new WaitForSeconds (delay);
			FindVisibleTargets ();
		}
	}

    void LateUpdate()
    {
        DrawFieldofView();
    }

    void FindVisibleTargets()
    {
		visibleTargets.Clear ();
		Collider[] targetsInViewRadius = Physics.OverlapSphere (transform.position, viewRadius, targetMask);

		for (int i = 0; i < targetsInViewRadius.Length; i++)
        {
			Transform target = targetsInViewRadius [i].transform;
			Vector3 dirToTarget = (target.position - transform.position).normalized;

			if (Vector3.Angle (transform.forward, dirToTarget) < viewAngle / 2)
            {
				float dstToTarget = Vector3.Distance (transform.position, target.position);

				if (!Physics.Raycast (transform.position, dirToTarget, dstToTarget, obstacleMask))
                {
					visibleTargets.Add (target);
				}
			}
		}
	}

    void DrawFieldofView()
    {
        int stepCount = Mathf.RoundToInt(viewAngle * meshResolution);   //ray count, number of rays sent out = resolution*angle
        float stepAngleSize = viewAngle / stepCount;                    //how many degrees in each step
        List<Vector3> viewPoints = new List<Vector3>();
        ViewCastInfo oldViewCast = new ViewCastInfo();                  //data from last raycast
        for(int i=0; i<=stepCount; i++)
        {
            float angle = transform.eulerAngles.y - viewAngle/2 + stepAngleSize*i;  //current angle = player current rotation, rotate back to lefmost view angle (-angle/2), the rotate to right most view angle???? (stepAS*i) of FOV maybe
            //Debug.DrawLine(transform.position, transform.position + DirFromAngle(angle, true) * viewRadius, Color.red); // draws raycast
            ViewCastInfo newViewCast = ViewCast(angle);                             //calls ViewCast function to find out if raycasts hit anything

            if (i > 0)
            {
                bool edgeDistanceThresholdExceeded = Mathf.Abs(oldViewCast.dst - newViewCast.dst) > edgeDistanceThreshold;
                if (oldViewCast.hit != newViewCast.hit || (oldViewCast.hit && newViewCast.hit && edgeDistanceThresholdExceeded))     //did the old view cast hit and the new one didnt? 
                {
                    EdgeInfo edge = FindEdge(oldViewCast, newViewCast);
                    if(edge.pointA != Vector3.zero)
                    {
                        viewPoints.Add(edge.pointA);
                    }
                    if (edge.pointB != Vector3.zero)
                    {
                        viewPoints.Add(edge.pointB);
                    }
                }                                        //or the old one didnt and the new one did.

            }

            viewPoints.Add(newViewCast.point);
            oldViewCast = newViewCast;
        }

        int vertexCount = viewPoints.Count + 1;
        Vector3[] vertices = new Vector3[vertexCount];
        int[] triangles = new int[(vertexCount - 2) * 3];

        vertices[0] = Vector3.zero;     //setting up first vertex
        for(int i=0; i<vertexCount-1; i++)
        {
            vertices[i + 1] = transform.InverseTransformPoint(viewPoints[i]) + Vector3.forward*maskCutawayDst;     //local space points

            if (i < vertexCount - 2)
            {
                triangles[i * 3] = 0;
                triangles[i * 3 + 1] = i + 1;
                triangles[i * 3 + 2] = i + 2;
            }
        }

        viewMesh.Clear();
        viewMesh.vertices = vertices;
        viewMesh.triangles = triangles;
        viewMesh.RecalculateNormals();
    }

    EdgeInfo FindEdge(ViewCastInfo minViewCast, ViewCastInfo maxViewCast)
    {
        float minAngle = minViewCast.angle;
        float maxAngle = maxViewCast.angle;
        Vector3 minPoint = Vector3.zero;
        Vector3 maxPoint = Vector3.zero;

        for (int i = 0; i < edgeResolveIterations; i++)
        {
            float angle = (minAngle + maxAngle) / 2;
            ViewCastInfo newViewCast = ViewCast(angle);
            bool edgeDistanceThresholdExceeded = Mathf.Abs(minViewCast.dst - newViewCast.dst) > edgeDistanceThreshold;
            if (newViewCast.hit == minViewCast.hit && !edgeDistanceThresholdExceeded)
            {
                minAngle = angle;
                minPoint = newViewCast.point;
            }
            else
            {
                maxAngle = angle;
                maxPoint = newViewCast.point;
            }

        }
        return new EdgeInfo(minPoint, maxPoint);
    }

    ViewCastInfo ViewCast(float globalAngle)
    {
        Vector3 dir = DirFromAngle(globalAngle, true);
        RaycastHit hit;
        
        if(Physics.Raycast(transform.position, dir, out hit, viewRadius, obstacleMask)) //if the ray cast hits something
        {
            return new ViewCastInfo(true, hit.point, hit.distance, globalAngle);        //return struct of hit data
        }
        else
        {
            return new ViewCastInfo(true, transform.position + dir * viewRadius, viewRadius, globalAngle); //return struct of not hit data
        }
    }

    public Vector3 DirFromAngle(float angleInDegrees, bool angleIsGlobal)
    {
		if (!angleIsGlobal)
        {
			angleInDegrees += transform.eulerAngles.y;
		}
		return new Vector3(Mathf.Sin(angleInDegrees * Mathf.Deg2Rad),0,Mathf.Cos(angleInDegrees * Mathf.Deg2Rad));
	}

    public struct ViewCastInfo  //stores data about raycasts
    {
        public bool hit;        //did the raycast hit something
        public Vector3 point;   //endpoint of ray
        public float dst;       //length of ray
        public float angle;     //angle ray was fired at

        public ViewCastInfo(bool _hit, Vector3 _point, float _dst, float _angle)    //constructor
        {
            hit = _hit;         
            point = _point;     
            dst = _dst;         
            angle = _angle;     
        
        }

    }

    public struct EdgeInfo  //struct for finding edges of objects
    {
        public Vector3 pointA;
        public Vector3 pointB;

        public EdgeInfo(Vector3 _pointA, Vector3 _pointB)
        {
            pointA = _pointA;
            pointB = _pointB;
        }
    }
}