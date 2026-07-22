using TS = Tekla.Structures;

namespace HideCurvedSheetMetalEdges
{
    using System.Collections.Generic;

    using TS.DrawingPresentationModel;
    using TS.Geometry3d;

    public partial class HideCurvedSheetMetalEdges
    {
        #region Graf linii – wykrywanie przekątnych

        private sealed class LineGraph
        {   
            public readonly List<Point> Nodes;
            public readonly Dictionary<int, List<int>> Adjacency;

            public LineGraph(
                List<Point> nodes,
                Dictionary<int, List<int>> adjacency)
            {
                Nodes = nodes;
                Adjacency = adjacency;
            }
        }

        private LineGraph BuildLineGraph(List<CachedLine> cachedLines)
        {
            double nodeTolerance = this.ActiveDrawingEpsilon;
            var nodes = new List<Point>();
            var adjacency = new Dictionary<int, List<int>>();

            foreach (var cl in cachedLines)
            {
                Point a = cl.Segment.Point1;
                Point b = cl.Segment.Point2;

                int ia = GetOrAddNode(a);
                int ib = GetOrAddNode(b);

                if (!adjacency[ia].Contains(ib))
                    adjacency[ia].Add(ib);
                if (!adjacency[ib].Contains(ia))
                    adjacency[ib].Add(ia);
            }

            return new LineGraph(nodes, adjacency);

            int GetOrAddNode(Point p)
            {
                for (int i = 0; i < nodes.Count; i++)
                {
                    if (Distance.PointToPoint(nodes[i], p) <= nodeTolerance)
                        return i;
                }

                nodes.Add(new Point(p.X, p.Y, 0));
                adjacency[nodes.Count - 1] = new List<int>();
                return nodes.Count - 1;
            }
        }

        private bool IsDiagonalInGraph(LinePrimitive linePrimitive, LineGraph graph)
        {
            if (graph == null)
                return false;

            double nodeTolerance = this.ActiveDrawingEpsilon;

            var pStart = new Point(linePrimitive.StartPoint.X, linePrimitive.StartPoint.Y, 0);
            var pEnd   = new Point(linePrimitive.EndPoint.X,   linePrimitive.EndPoint.Y,   0);

            int start = -1;
            int end   = -1;

            for (int i = 0; i < graph.Nodes.Count; i++)
            {
                var n = graph.Nodes[i];

                if (start == -1 && Distance.PointToPoint(n, pStart) <= nodeTolerance)
                    start = i;
    
                if (end == -1 && Distance.PointToPoint(n, pEnd) <= nodeTolerance)
                    end = i;

                if (start != -1 && end != -1)
                    break;
            }

            if (start == -1 || end == -1)
                return false;

            if (!graph.Adjacency.TryGetValue(start, out var startNeighbors) ||
                !graph.Adjacency.TryGetValue(end,   out var endNeighbors))
                return false;

            if (startNeighbors.Count < 2 || endNeighbors.Count < 2)
                return false;

            var visited = new HashSet<int> { start };
            var queue   = new Queue<int>();
            queue.Enqueue(start);

            while (queue.Count > 0)
            {
                int v = queue.Dequeue();

                if (!graph.Adjacency.TryGetValue(v, out var neighbors))
                    continue;

                foreach (int w in neighbors)
                {
                    if ((v == start && w == end) || (v == end && w == start))
                        continue;

                    if (!visited.Add(w))
                        continue;

                    if (w == end)
                        return true;

                    queue.Enqueue(w);
                }
            }

            return false;
        }

        #endregion
    }
}
