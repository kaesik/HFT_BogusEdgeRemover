using TS = Tekla.Structures;
using TSM = Tekla.Structures.Model;
using TSD = Tekla.Structures.Drawing;
using TSP = Tekla.Structures.Plugins;

namespace HideCurvedSheetMetalEdges
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel.Composition;
    using System.Linq;

    using Tekla.Common.Geometry;
    using TS.DrawingPresentationModel;
    using TS.DrawingPresentationPluginInterface;
    using TS.Geometry3d;
    using TS.Solid;

    public partial class HideCurvedSheetMetalEdges
    {
        #region Cięcie linii po przecięciach (snap)

        private static List<LinePrimitive> SplitLinePrimitiveByIntersections(
            LinePrimitive baseLine,
            List<CachedLine> cachedLines)
        {
            var baseSeg = new LineSegment(
                new Point(baseLine.StartPoint.X, baseLine.StartPoint.Y, 0),
                new Point(baseLine.EndPoint.X,   baseLine.EndPoint.Y,   0));

            var cutters = new List<LineSegment>(cachedLines.Count);
            foreach (var cl in cachedLines)
            {
                if (AreSameLinePrimitives(cl.Primitive, baseLine))
                    continue;

                cutters.Add(cl.Segment);
            }

            var splitSegs = SplitLineByIntersections(baseSeg, cutters);

            var result = new List<LinePrimitive>(splitSegs.Count);
            foreach (var s in splitSegs)
            {
                result.Add(new LinePrimitive(
                    new Vector2(s.Point1.X, s.Point1.Y),
                    new Vector2(s.Point2.X, s.Point2.Y)));
            }

            return result;
        }

        private static List<LineSegment> SplitLineByIntersections(
            LineSegment baseSeg,
            List<LineSegment> cutters)
        {
            var points = new List<Point> { baseSeg.Point1, baseSeg.Point2 };

            foreach (var cutter in cutters)
            {
                var p = SegmentIntersectionSnap(
                    baseSeg.Point1, baseSeg.Point2,
                    cutter.Point1, cutter.Point2);

                if (p != null)
                    points.Add(p);
            }

            points = UniquePoints(points, DrawingEpsilon);

            points.Sort((a, b) =>
                Distance.PointToPoint(baseSeg.Point1, a)
                    .CompareTo(Distance.PointToPoint(baseSeg.Point1, b)));

            var result = new List<LineSegment>();
            for (int i = 0; i < points.Count - 1; i++)
            {
                if (Distance.PointToPoint(points[i], points[i + 1]) > DrawingEpsilon)
                    result.Add(new LineSegment(points[i], points[i + 1]));
            }

            return result;
        }

        private static Point SegmentIntersectionSnap(Point p1, Point p2, Point p3, Point p4)
        {
            var exact = SegmentIntersectionExact(p1, p2, p3, p4);
            if (exact != null)
                return exact;

            var c12 = ClosestPointsBetweenSegments2D(p1, p2, p3, p4);
            if (c12 == null)
                return null;

            var a = c12.Value.A;
            var b = c12.Value.B;

            return Distance.PointToPoint(a, b) <= DrawingEpsilon
                ? new Point((a.X + b.X) / 2.0, (a.Y + b.Y) / 2.0, 0.0)
                : null;
        }

        private static Point SegmentIntersectionExact(Point p1, Point p2, Point p3, Point p4)
        {
            double x1 = p1.X, y1 = p1.Y;
            double x2 = p2.X, y2 = p2.Y;
            double x3 = p3.X, y3 = p3.Y;
            double x4 = p4.X, y4 = p4.Y;

            double denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
            if (Math.Abs(denom) < DrawingEpsilon)
                return null;

            double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;
            double u = ((x1 - x3) * (y1 - y2) - (y1 - y3) * (x1 - x2)) / denom;

            if (t is < -DrawingEpsilon or > 1 + DrawingEpsilon) return null;
            if (u is < -DrawingEpsilon or > 1 + DrawingEpsilon) return null;

            return new Point(x1 + t * (x2 - x1), y1 + t * (y2 - y1), 0.0);
        }

        private static (Point A, Point B)? ClosestPointsBetweenSegments2D(Point p1, Point p2, Point p3, Point p4)
        {
            Point aOn34 = ClosestPointOnSegment2D(p3, p4, p1);
            Point bOn34 = ClosestPointOnSegment2D(p3, p4, p2);
            Point aOn12 = ClosestPointOnSegment2D(p1, p2, p3);
            Point bOn12 = ClosestPointOnSegment2D(p1, p2, p4);

            var candidates = new List<(Point A, Point B)>
            {
                (p1, aOn34),
                (p2, bOn34),
                (aOn12, p3),
                (bOn12, p4)
            };

            double best = double.MaxValue;
            (Point A, Point B) bestPair = default;

            foreach (var c in candidates)
            {
                double d = Distance.PointToPoint(c.A, c.B);
                if (d < best)
                {
                    best = d;
                    bestPair = c;
                }
            }

            return bestPair;
        }

        private static Point ClosestPointOnSegment2D(Point a, Point b, Point p)
        {
            double ax = a.X, ay = a.Y;
            double bx = b.X, by = b.Y;
            double px = p.X, py = p.Y;

            double abx = bx - ax;
            double aby = by - ay;
            double abLen2 = abx * abx + aby * aby;

            if (abLen2 < 1e-12)
                return new Point(ax, ay, 0);

            double t = ((px - ax) * abx + (py - ay) * aby) / abLen2;
            t = Math.Max(0.0, Math.Min(1.0, t));

            return new Point(ax + t * abx, ay + t * aby, 0);
        }

        private static List<Point> UniquePoints(List<Point> points, double eps)
        {
            var unique = new List<Point>();
            foreach (var p in points)
            {
                if (!unique.Any(u => Distance.PointToPoint(u, p) <= eps))
                    unique.Add(p);
            }
            return unique;
        }

        private static bool AreSameLinePrimitives(LinePrimitive a, LinePrimitive b)
        {
            bool sameDir =
                a.StartPoint.DistanceTo(b.StartPoint) < DrawingEpsilon &&
                a.EndPoint.DistanceTo(b.EndPoint)     < DrawingEpsilon;

            bool oppositeDir =
                a.StartPoint.DistanceTo(b.EndPoint)   < DrawingEpsilon &&
                a.EndPoint.DistanceTo(b.StartPoint)   < DrawingEpsilon;

            return sameDir || oppositeDir;
        }

        #endregion
    }
}
