using GameFW.Nav._2DGraph;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace GameFW.Nav
{
    public abstract class PathFinder:IPathFind
    {
        private static PathFinder pathFind = PathFinderFileReader.Read(new StringBuilder(System.IO.Directory.GetCurrentDirectory()).Append("/Data/Fight_NavMesh_Origin.obj").ToString());

        public static PathFinder Instance {
            get {
                return pathFind;
            }
        }


        protected List<Vector3> points;
        protected Dictionary<int, Poly> polys;
        protected Dictionary<int, List<NodeEdge>> edges;
        protected List<int>[,] paths;
        protected NavAOI navAOI;
        protected bool isPathPreCalculate;
        protected int componentCount;
        protected int nodeCount;

        public PathFinder(List<Vector3> points, Dictionary<int, Poly> polys, Dictionary<int, List<NodeEdge>> edges, float left, float bottom, float tileSize, int width, int height, int componentCount, List<int>[,] aoiList, List<int>[,] pths = null) {
            this.points = points;
            this.edges = edges;
            this.polys = polys;
            this.componentCount = componentCount;
            this.nodeCount = polys.Count;

            navAOI = new NavAOI(left, bottom, width, height, tileSize, aoiList);

            this.isPathPreCalculate = (pths != null);
            if (pths != null)
            {
                this.paths = pths;
            }
            else {
                this.paths = new List<int>[polys.Count, polys.Count];
                for (int i = 0; i < polys.Count; i++) {
                    for (int j = 0; j < polys.Count; j++) {
                        paths[i, j] = null;
                    }
                }
            }
        }

        public abstract List<int> GetPaths(Vector2 pos, Vector2 target);


        protected int GetPoly(Vector2 pos, List<int> plys) {
            foreach (int i in plys) {
                Poly p = polys[i];
                if (GraphTester2D.IsInside(pos, p.GetGeo2D()))
                    return i;
            }

            return -1;
        }

        public int GetPolyByPos(Vector2 pos) {
            List<int> plys = navAOI.GetPolyListInAOI(pos.x, pos.y);

            return GetPoly(pos, plys);
        }

        public Vector3 GetMovePoint(int from, int to) {
            if (edges.ContainsKey(from)) {
                List<NodeEdge> edgeList = edges[from];

                foreach (NodeEdge edge in edgeList) {
                    if (edge.Other(from) == to) {
                        Vector3 center = (points[edge.PointA] + points[edge.PointB]) * 0.5f;
                        return center;
                    }
                }
            }
            return Vector3.zero;
        }
    }
}
