using TS = Tekla.Structures;

namespace HideCurvedSheetMetalEdges
{
    using System.Collections.Generic;

    using TS.DrawingPresentationModel;
    using TS.Geometry3d;

    public partial class HideCurvedSheetMetalEdges
    {
        #region Klasyfikacja: krawędź wewnętrzna / zewnętrzna

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

                if (SideOk(top) && SideOk(bottom) && SideOk(right) && SideOk(left))
                    return true;
            }

            return SideOk(top) && SideOk(bottom) && SideOk(right) && SideOk(left);

            static bool SideOk(LineIntersections s)
            {
                int hits = (s.Line01 ? 1 : 0) + (s.Line05 ? 1 : 0) + (s.Line09 ? 1 : 0);
                return hits >= 2;
            }
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
    }
}
