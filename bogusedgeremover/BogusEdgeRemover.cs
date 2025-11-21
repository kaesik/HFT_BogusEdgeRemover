using TS = Tekla.Structures;
using TSM = Tekla.Structures.Model;
using TSD = Tekla.Structures.Drawing;
using TSP = Tekla.Structures.Plugins;

namespace BogusEdgeRemover
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

    [Export(typeof(IDrawingPresentationPlugin))]
    [ExportMetadata("ObjectType", new[]
    {
        CustomPresentationObjectTypesEnum.Pours,
        CustomPresentationObjectTypesEnum.Parts
    })]
    [ExportMetadata("BriefDescription", "Bogus Edge Remover")]
    [ExportMetadata("Description", "Presentation plugin that removes the edges of a part that are a result of a non-planar surface.")]
    [ExportMetadata("GUID", "00CE0BCD-429B-48AC-A235-DC14311204D4")]
    public class BogusEdgeRemover : IDrawingPresentationPlugin
    {
        #region Stałe / pola

        private const double ModelEpsilon        = 1.0;
        private const double DrawingEpsilon      = 0.0001;
        private const double Degrees180          = Math.PI;
        private const double Degrees90           = Math.PI / 2;
        private const double BigAngleAllowance   = Math.PI / 2.5;
        private const double SmallAngleAllowance = Math.PI / 32;
        private const double AngleEpsilon        = 0.001;

        private readonly Vector _globalAxisZ = new(0.0, 0.0, 1.0);
        private readonly TSM.Model _model = new();

        private double Scale { get; set; }
        private Matrix TransformationMatrix { get; set; }

        #endregion

        #region Entry point

        public Segment CreatePresentation(Segment presentation)
        {
            if (presentation == null)
                return null;

            var drawingPart = GetDrawingPart(presentation.Id);
            if (drawingPart == null)
                return presentation;

            if (_model.SelectModelObject(drawingPart.ModelIdentifier) is not TSM.Part modelPart)
                return presentation;

            if (drawingPart.GetView() is not TSD.View view)
                return presentation;

            Scale = view.Attributes.Scale;
            TransformationMatrix = MatrixFactory.ToCoordinateSystem(view.DisplayCoordinateSystem);

            Vector viewAxisX = view.ViewCoordinateSystem.AxisX;
            Vector viewAxisY = view.ViewCoordinateSystem.AxisY;
            Vector viewAxisZ = viewAxisX.Cross(viewAxisY);

            var edgesToDelete = GetModelEdgesInDrawingToBeDeletedInDrawing(modelPart, viewAxisZ);
            if (edgesToDelete.Count > 0)
                RemoveBogusLines(presentation, edgesToDelete);

            return presentation;
        }

        #endregion

        #region Główna logika usuwania / dzielenia

        private void RemoveBogusLines(Segment presentation, List<ModelEdgePair> modelEdgesToBeDeleted)
        {
            if (presentation?.Primitives == null || presentation.Primitives.Count == 0)
                return;

            var cachedLines = BuildCachedLines(presentation);

            int removedHiddenLinesCount = 0;

            foreach (var primitive in presentation.Primitives)
            {
                if (primitive is PrimitiveGroup group)
                {
                    RemoveBogusLinesInPrimitiveGroup(
                        modelEdgesToBeDeleted,
                        group,
                        cachedLines,
                        ref removedHiddenLinesCount);
                }
            }
        }

        private void RemoveBogusLinesInPrimitiveGroup(
            List<ModelEdgePair> modelEdgesToBeDeleted,
            PrimitiveGroup primitiveGroup,
            List<CachedLine> cachedLines,
            ref int removedHiddenLinesCount)
        {
            if (primitiveGroup?.Primitives == null || primitiveGroup.Primitives.Count == 0)
                return;

            var newPrimitives = new List<PrimitiveBase>(primitiveGroup.Primitives.Count);
            bool fullLinePrimitiveGroup = primitiveGroup.Pen.LineType == 1;

            foreach (PrimitiveBase primitiveBase in primitiveGroup.Primitives)
            {
                switch (primitiveBase)
                {
                    case LinePrimitive linePrimitive:
                    {
                        var splitLines = SplitLinePrimitiveByIntersections(linePrimitive, cachedLines);

                        foreach (var splitLine in splitLines)
                        {
                            if (!ShouldDeleteLine(
                                    splitLine,
                                    modelEdgesToBeDeleted,
                                    fullLinePrimitiveGroup,
                                    cachedLines,
                                    ref removedHiddenLinesCount))
                            {
                                newPrimitives.Add(splitLine);
                            }
                        }

                        break;
                    }

                    case PrimitiveGroup nestedGroup:
                    {
                        RemoveBogusLinesInPrimitiveGroup(
                            modelEdgesToBeDeleted,
                            nestedGroup,
                            cachedLines,
                            ref removedHiddenLinesCount);

                        newPrimitives.Add(nestedGroup);
                        break;
                    }

                    default:
                        newPrimitives.Add(primitiveBase);
                        break;
                }
            }

            primitiveGroup.Primitives.Clear();
            foreach (var primitive in newPrimitives)
                primitiveGroup.Primitives.Add(primitive);
        }

        private bool ShouldDeleteLine(
            LinePrimitive linePrimitive,
            List<ModelEdgePair> modelEdgesToBeDeleted,
            bool fullLinePrimitiveGroup,
            List<CachedLine> cachedLines,
            ref int removedHiddenLinesCount)
        {
            if (!LinePrimitiveShouldBeDeleted(linePrimitive, fullLinePrimitiveGroup, modelEdgesToBeDeleted))
                return false;

            if (!LinePrimitiveIsNotExternal(linePrimitive, cachedLines))
                return false;

            removedHiddenLinesCount++;
            return true;
        }

        #endregion

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

        #region Wewnętrzna / zewnętrzna krawędź (na cache)

        private bool LinePrimitiveIsNotExternal(LinePrimitive linePrimitive, List<CachedLine> cachedLines)
        {
            if (cachedLines == null || cachedLines.Count == 0)
                return true;

            LineIntersections top    = new(false, false, false);
            LineIntersections bottom = new(false, false, false);
            LineIntersections right  = new(false, false, false);
            LineIntersections left   = new(false, false, false);

            GetVerticalAndHorizontalCenterLines(linePrimitive, 0.25, out var v01, out var h01, out _);
            GetVerticalAndHorizontalCenterLines(linePrimitive, 0.5,  out var v05, out var h05, out var centerPoint);
            GetVerticalAndHorizontalCenterLines(linePrimitive, 0.75, out var v09, out var h09, out _);

            foreach (var cl in cachedLines)
            {
                var current = cl.Primitive;
                if (current == null || AreSameLinePrimitives(current, linePrimitive))
                    continue;

                LineOrArcPrimitiveIsNotExternal(
                    v01, h01,
                    v05, h05,
                    v09, h09,
                    centerPoint,
                    current,
                    null,
                    ref top, ref bottom, ref right, ref left);
            }

            return
                top    is { Line01: true, Line05: true, Line09: true } &&
                bottom is { Line01: true, Line05: true, Line09: true } &&
                right  is { Line01: true, Line05: true, Line09: true } &&
                left   is { Line01: true, Line05: true, Line09: true };
        }

        private static void LineOrArcPrimitiveIsNotExternal(
            List<Line> verticalLine01,
            List<Line> horizontalLine01,
            List<Line> verticalLine05,
            List<Line> horizontalLine05,
            List<Line> verticalLine09,
            List<Line> horizontalLine09,
            Point centerPoint,
            LinePrimitive currentLinePrimitive,
            ArcPrimitive currentArcPrimitive,
            ref LineIntersections top,
            ref LineIntersections bottom,
            ref LineIntersections right,
            ref LineIntersections left)
        {
            Point currentLineStartPoint = null;
            Point currentLineEndPoint = null;

            if (currentLinePrimitive != null)
            {
                currentLineStartPoint = new Point(currentLinePrimitive.StartPoint.X, currentLinePrimitive.StartPoint.Y);
                currentLineEndPoint   = new Point(currentLinePrimitive.EndPoint.X,   currentLinePrimitive.EndPoint.Y);
            }
            else if (currentArcPrimitive != null)
            {
                currentLineStartPoint = new Point(currentArcPrimitive.StartPoint.X, currentArcPrimitive.StartPoint.Y);
                currentLineEndPoint   = new Point(currentArcPrimitive.EndPoint.X,   currentArcPrimitive.EndPoint.Y);
            }

            if (currentLineStartPoint == null)
                return;

            Line currentLine = new(currentLineStartPoint, currentLineEndPoint);

            CheckWhereLinesIntersect(verticalLine01,   true,  centerPoint, currentLineStartPoint, currentLineEndPoint, currentLine, ref top.Line01, ref bottom.Line01, ref right.Line01, ref left.Line01);
            CheckWhereLinesIntersect(horizontalLine01, false, centerPoint, currentLineStartPoint, currentLineEndPoint, currentLine, ref top.Line01, ref bottom.Line01, ref right.Line01, ref left.Line01);

            CheckWhereLinesIntersect(verticalLine05,   true,  centerPoint, currentLineStartPoint, currentLineEndPoint, currentLine, ref top.Line05, ref bottom.Line05, ref right.Line05, ref left.Line05);
            CheckWhereLinesIntersect(horizontalLine05, false, centerPoint, currentLineStartPoint, currentLineEndPoint, currentLine, ref top.Line05, ref bottom.Line05, ref right.Line05, ref left.Line05);

            CheckWhereLinesIntersect(verticalLine09,   true,  centerPoint, currentLineStartPoint, currentLineEndPoint, currentLine, ref top.Line09, ref bottom.Line09, ref right.Line09, ref left.Line09);
            CheckWhereLinesIntersect(horizontalLine09, false, centerPoint, currentLineStartPoint, currentLineEndPoint, currentLine, ref top.Line09, ref bottom.Line09, ref right.Line09, ref left.Line09);
        }

        private static void CheckWhereLinesIntersect(
            List<Line> lineCollection,
            bool vertical,
            Point centerPoint,
            Point currentLineStartPoint,
            Point currentLineEndPoint,
            Line currentLine,
            ref bool top,
            ref bool bottom,
            ref bool right,
            ref bool left)
        {
            foreach (Line line in lineCollection)
            {
                LineSegment intersection = Intersection.LineToLine(currentLine, line);

                if (intersection == null) continue;
                if (intersection.Length() >= DrawingEpsilon) continue;
                if (Distance.PointToPoint(currentLineStartPoint, intersection.Point1) <= DrawingEpsilon) continue;
                if (Distance.PointToPoint(currentLineEndPoint,   intersection.Point1) <= DrawingEpsilon) continue;

                if (new Vector(currentLineStartPoint - intersection.Point1)
                        .GetAngleBetween(new Vector(currentLineEndPoint - intersection.Point1)) <= Degrees90)
                    continue;

                if (vertical)
                {
                    if (intersection.Point1.Y > centerPoint.Y + DrawingEpsilon) top = true;
                    else if (intersection.Point1.Y < centerPoint.Y - DrawingEpsilon) bottom = true;
                }
                else
                {
                    if (intersection.Point1.X > centerPoint.X + DrawingEpsilon) right = true;
                    else if (intersection.Point1.X < centerPoint.X - DrawingEpsilon) left = true;
                }
            }
        }

        private static void GetVerticalAndHorizontalCenterLines(
            LinePrimitive linePrimitive,
            double lineRatio,
            out List<Line> verticalLine,
            out List<Line> horizontalLine,
            out Point centerPoint)
        {
            verticalLine = new List<Line>();
            horizontalLine = new List<Line>();

            centerPoint = new Point(
                lineRatio * linePrimitive.StartPoint.X + (1 - lineRatio) * linePrimitive.EndPoint.X,
                lineRatio * linePrimitive.StartPoint.Y + (1 - lineRatio) * linePrimitive.EndPoint.Y);

            verticalLine.Add(new Line(centerPoint, new Vector(new Point(0.0, 1.0))));
            verticalLine.Add(new Line(centerPoint, new Vector(new Point(1.0, 3.5))));
            verticalLine.Add(new Line(centerPoint, new Vector(new Point(-1.0, 3.5))));

            horizontalLine.Add(new Line(centerPoint, new Vector(new Point(1.0, 0.0))));
            horizontalLine.Add(new Line(centerPoint, new Vector(new Point(3.5, 1.0))));
            horizontalLine.Add(new Line(centerPoint, new Vector(new Point(3.5, -1.0))));
        }

        #endregion

        #region Dopasowanie linii rysunku do krawędzi modelu

        private static bool LinePrimitiveShouldBeDeleted(
            LinePrimitive linePrimitive,
            bool fullLinePrimitiveGroup,
            List<ModelEdgePair> modelEdgesToBeDeleted)
        {
            foreach (ModelEdgePair modelEdgeToBeDeleted in modelEdgesToBeDeleted)
            {
                if (!LinePrimitiveOverlapsWithEdgeToBeDeleted(linePrimitive, modelEdgeToBeDeleted))
                    continue;

                if (modelEdgeToBeDeleted.VisibleLine || fullLinePrimitiveGroup == false)
                    return true;
            }

            return false;
        }

        private static bool LinePrimitiveOverlapsWithEdgeToBeDeleted(
            LinePrimitive linePrimitive,
            ModelEdgePair modelEdgeToBeDeleted)
        {
            var edge = modelEdgeToBeDeleted.ModelEdgeInDrawing;

            Line edgeLine = new(
                new Point(edge.StartPoint.X, edge.StartPoint.Y),
                new Point(edge.EndPoint.X,   edge.EndPoint.Y));

            var lpStart = new Point(linePrimitive.StartPoint.X, linePrimitive.StartPoint.Y);
            var lpEnd   = new Point(linePrimitive.EndPoint.X,   linePrimitive.EndPoint.Y);

            if (Distance.PointToLine(lpStart, edgeLine) > DrawingEpsilon ||
                Distance.PointToLine(lpEnd,   edgeLine) > DrawingEpsilon)
                return false;

            var e0 = new Point(edge.StartPoint.X, edge.StartPoint.Y);
            var e1 = new Point(edge.EndPoint.X,   edge.EndPoint.Y);

            var edgeDir = new Vector(e1.X - e0.X, e1.Y - e0.Y, 0);
            double edgeLen = edgeDir.GetLength();
            if (edgeLen < DrawingEpsilon)
                return false;

            var edgeDirUnit = new Vector(edgeDir);
            edgeDirUnit.Normalize();

            double tLp0 = Vector.Dot(new Vector(lpStart.X - e0.X, lpStart.Y - e0.Y, 0), edgeDirUnit);
            double tLp1 = Vector.Dot(new Vector(lpEnd.X   - e0.X, lpEnd.Y   - e0.Y, 0), edgeDirUnit);

            double lpMin = Math.Min(tLp0, tLp1);
            double lpMax = Math.Max(tLp0, tLp1);

            double overlapMin = Math.Max(lpMin, 0.0);
            double overlapMax = Math.Min(lpMax, edgeLen);

            if (overlapMax - overlapMin <= DrawingEpsilon)
                return false;

            double overlapLen = overlapMax - overlapMin;
            double lpLen = Distance.PointToPoint(lpStart, lpEnd);

            return overlapLen >= lpLen * 0.8;
        }

        #endregion

        #region Analiza bryły modelu

        private List<ModelEdgePair> GetModelEdgesInDrawingToBeDeletedInDrawing(TSM.Part selectedModelPart, Vector viewAxisZ)
        {
            var modelEdgesInDrawing = new List<ModelEdgePair>();

            TSM.Solid solid = selectedModelPart.GetSolid();
            FaceEnumerator faceEnum = solid.GetFaceEnumerator();

            while (faceEnum.MoveNext())
            {
                if (faceEnum.Current is not { } currentFace)
                    continue;

                if (!FaceIsNotHorizontalNorVertical(currentFace))
                    continue;

                var facesWithSimilarNormal = GetFacesWithSimilarNormal(currentFace, solid.GetFaceEnumerator());

                foreach (Face faceWithSimilarNormal in facesWithSimilarNormal)
                {
                    LineSegment commonEdge = GetCommonEdge(currentFace, faceWithSimilarNormal);
                    if (commonEdge == null)
                        continue;

                    Point transformedStartPoint = TransformationMatrix.Transform(commonEdge.StartPoint);
                    Point transformedEndPoint   = TransformationMatrix.Transform(commonEdge.EndPoint);

                    var commonEdgeInDrawing = new LinePrimitive(
                        new Vector2(transformedStartPoint.X / Scale, transformedStartPoint.Y / Scale),
                        new Vector2(transformedEndPoint.X   / Scale, transformedEndPoint.Y   / Scale));

                    Vector middleNormal = new(
                        (currentFace.Normal.X + faceWithSimilarNormal.Normal.X) / 2.0,
                        (currentFace.Normal.Y + faceWithSimilarNormal.Normal.Y) / 2.0,
                        (currentFace.Normal.Z + faceWithSimilarNormal.Normal.Z) / 2.0);

                    bool visibleLine = middleNormal.GetAngleBetween(viewAxisZ) < Degrees90 + SmallAngleAllowance;

                    if (!CommonEdgeIsPresentInModelEdges(commonEdgeInDrawing, visibleLine, modelEdgesInDrawing))
                        modelEdgesInDrawing.Add(new ModelEdgePair(commonEdgeInDrawing, visibleLine));
                }
            }

            return modelEdgesInDrawing;
        }

        private bool FaceIsNotHorizontalNorVertical(Face currentFace)
        {
            double faceAngle = currentFace.Normal.GetAngleBetween(_globalAxisZ);

            if (faceAngle < AngleEpsilon)
                return false;

            if (Math.Abs(faceAngle - Degrees90) < AngleEpsilon)
                return false;

            return !(faceAngle > Degrees180 - AngleEpsilon);
        }

        private static List<Face> GetFacesWithSimilarNormal(Face currentFace, FaceEnumerator faceEnumerator)
        {
            var facesWithSimilarNormal = new List<Face>();

            while (faceEnumerator.MoveNext())
            {
                if (faceEnumerator.Current is not { } secondaryFace)
                    continue;

                if (secondaryFace.Equals(currentFace))
                    continue;

                if (currentFace.Normal.GetAngleBetween(secondaryFace.Normal) < BigAngleAllowance &&
                    secondaryFace.Normal.GetAngleBetween(new Vector(0,0,1)) > AngleEpsilon) // szybki filtr poziom/pion
                {
                    facesWithSimilarNormal.Add(secondaryFace);
                }
            }

            return facesWithSimilarNormal;
        }

        private static LineSegment GetCommonEdge(Face currentFace, Face faceWithSimilarNormal)
        {
            var commonVertexes = new List<Point>();

            var currentFaceVertexes = GetFaceVertexes(currentFace);
            var similarNormalVertexes = GetFaceVertexes(faceWithSimilarNormal);

            foreach (Point currentVertex in currentFaceVertexes)
            {
                foreach (Point similarNormalVertex in similarNormalVertexes)
                {
                    if (Distance.PointToPoint(currentVertex, similarNormalVertex) < ModelEpsilon)
                        commonVertexes.Add(currentVertex);
                }
            }

            return commonVertexes.Count == 2
                ? new LineSegment(commonVertexes[0], commonVertexes[1])
                : null;
        }

        private static List<Point> GetFaceVertexes(Face currentFace)
        {
            var faceVertexes = new List<Point>();
            LoopEnumerator loopEnum = currentFace.GetLoopEnumerator();

            while (loopEnum.MoveNext())
            {
                if (loopEnum.Current is not { } loop)
                    continue;

                VertexEnumerator vertexEnum = loop.GetVertexEnumerator();
                while (vertexEnum.MoveNext())
                {
                    Point vertex = vertexEnum.Current;
                    if (vertex != null)
                        faceVertexes.Add(vertex);
                }
            }

            return faceVertexes;
        }

        private static bool CommonEdgeIsPresentInModelEdges(
            LinePrimitive commonEdgeInDrawing,
            bool visibleLine,
            List<ModelEdgePair> modelEdgesInDrawing)
        {
            foreach (ModelEdgePair modelEdge in modelEdgesInDrawing)
            {
                bool sameEndPoints =
                    (commonEdgeInDrawing.StartPoint.DistanceTo(modelEdge.ModelEdgeInDrawing.StartPoint) < DrawingEpsilon &&
                     commonEdgeInDrawing.EndPoint.DistanceTo(modelEdge.ModelEdgeInDrawing.EndPoint)   < DrawingEpsilon)
                    || (commonEdgeInDrawing.StartPoint.DistanceTo(modelEdge.ModelEdgeInDrawing.EndPoint) < DrawingEpsilon &&
                        commonEdgeInDrawing.EndPoint.DistanceTo(modelEdge.ModelEdgeInDrawing.StartPoint) < DrawingEpsilon);

                if (sameEndPoints && modelEdge.VisibleLine == visibleLine)
                    return true;
            }

            return false;
        }

        #endregion

        #region Pomocnicze (Drawing / cache)

        private static TSD.Part GetDrawingPart(int drawingId)
        {
            var identifier = new TS.Identifier(drawingId);
            var input = new TSP.DrawingPluginBase.InputDefinition(identifier, identifier);
            return TSD.Tools.InputDefinitionFactory.GetDrawingObject(input) as TSD.Part;
        }

        private static List<CachedLine> BuildCachedLines(Segment presentation)
        {
            var cachedLines = new List<CachedLine>();

            foreach (var primitive in presentation.Primitives)
            {
                if (primitive is PrimitiveGroup g)
                    CollectAllLinesFromGroup(g, cachedLines);
            }

            return cachedLines;
        }

        private readonly struct CachedLine
        {
            public readonly LinePrimitive Primitive;
            public readonly LineSegment Segment;

            public CachedLine(LinePrimitive primitive)
            {
                Primitive = primitive;
                Segment = new LineSegment(
                    new Point(primitive.StartPoint.X, primitive.StartPoint.Y, 0),
                    new Point(primitive.EndPoint.X,   primitive.EndPoint.Y,   0)
                );
            }
        }

        private static void CollectAllLinesFromGroup(PrimitiveGroup group, List<CachedLine> output)
        {
            if (group?.Primitives == null) return;

            foreach (var p in group.Primitives)
            {
                switch (p)
                {
                    case LinePrimitive lp:
                        output.Add(new CachedLine(lp));
                        break;

                    case PrimitiveGroup nested:
                        CollectAllLinesFromGroup(nested, output);
                        break;
                }
            }
        }

        private readonly struct ModelEdgePair
        {
            public readonly LinePrimitive ModelEdgeInDrawing;
            public readonly bool VisibleLine;

            public ModelEdgePair(LinePrimitive modelEdgeInDrawing, bool visibleLine)
            {
                ModelEdgeInDrawing = modelEdgeInDrawing;
                VisibleLine = visibleLine;
            }
        }

        private struct LineIntersections
        {
            public bool Line01;
            public bool Line05;
            public bool Line09;

            public LineIntersections(bool line01, bool line05, bool line09)
            {
                Line01 = line01;
                Line05 = line05;
                Line09 = line09;
            }
        }

        #endregion
    }
}
