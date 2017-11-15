using CommonTools;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace GameFW.Nav
{
    public class PathFinderRealtime : PathFinder
    {
        public PathFinderRealtime(List<Vector3> points, Dictionary<int, Poly> polys, Dictionary<int, List<NodeEdge>> edges, float left, float bottom, float tileSize, int width, int height, int componentCount, List<int>[,] aoiList, List<int>[,] pths = null) : base(points, polys, edges, left, bottom, tileSize, width, height, componentCount, aoiList, pths)
        {
            realDist = new List<float>(nodeCount);
            from = new List<int>(nodeCount);
            marked = new List<bool>(nodeCount);
            queue = new RBTree<float, int>();
        }

        public override List<int> GetPaths(Vector2 pos, Vector2 target)
        {
            int tarNode = GetPoly(target, navAOI.GetPolyListInAOI(target.x, target.y));
            int srcNode = GetPoly(pos, navAOI.GetPolyListInAOI(pos.x, pos.y));

            if (tarNode == -1 || srcNode == -1) {
                Console.WriteLine("导航错误: srcPos:" + pos.ToString() + ";tarPos:"+target.ToString() + ". srcIndex:"+srcNode +";tarIndex:"+tarNode);
                return null;
            }
            if (paths[srcNode, tarNode] == null) {
                paths[srcNode, tarNode] = PathAStar(srcNode, tarNode);
                for (int i = 1; i < paths[srcNode, tarNode].Count; i++) {
                    int n = paths[srcNode, tarNode].Count;
                    List<int> pth = new List<int>(n - i);
                    for (int j = i; j < n; j++)
                        pth.Add(paths[srcNode, tarNode][j]);

                    paths[paths[srcNode, tarNode][i], tarNode] = pth;
                }
            }

            return paths[srcNode, tarNode];
        }

        List<float> realDist;
        List<int> from;
        List<bool> marked;
        RBTree<float, int> queue;

        private List<int> PathAStar(int sourceNode, int targetNode) {
            realDist.Clear();
            from.Clear();
            marked.Clear();
            for (int i = 0; i < polys.Count; i++) {
                realDist.Add(float.MaxValue);
                from.Add(-1);
                marked.Add(false);
            }

            realDist[sourceNode] = 0f;
            from[sourceNode] = sourceNode;
            queue.Clear();
            queue.Add(Vector2.SqrMagnitude(polys[sourceNode].Center - polys[targetNode].Center), sourceNode);
            marked[sourceNode] = true;
            while (queue.Count > 0) {
                KeyValuePair<float, int> pair = queue.Min();
                int v = pair.Value;
                if (v == targetNode)
                    break;

                queue.Remove(pair.Key);
                marked[v] = true;

                foreach (NodeEdge e in edges[v]) {
                    int w = e.Other(v);
                    if (!marked[w])
                    {
                        float HCost = Vector2.SqrMagnitude(polys[targetNode].Center - polys[w].Center);
                        float GCost = realDist[v] + e.Weight;

                        if (GCost < realDist[w] || from[w] == -1) {
                            realDist[w] = GCost;
                            from[w] = v;

                            if (!queue.ContainsValue(w))
                                queue.Add(HCost + GCost, w);
                            else {
                                queue.Remove(queue.KeyOfValue(w));
                                queue.Add(HCost+GCost, w);
                            }
                        }
                    }
                }
            }

            return Path(sourceNode, targetNode, from);
        }

        /// <summary>
        /// 得到从s到d的路径
        /// </summary>
        /// <param name="s"></param>
        /// <param name="d"></param>
        /// <param name="from"></param>
        /// <returns></returns>
        private List<int> Path(int s, int d, List<int> from)
        {
            if (from[d] == -1)
                return null;

            List<int> pth = new List<int>();
            while (d != s)
            {
                pth.Add(d);
                d = from[d];
            }
            pth.Add(s);

            pth.Reverse();
            return pth;
        }
    }
}
