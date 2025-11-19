using System.ComponentModel.Composition;
using Tekla.Structures.DrawingPresentationModel;
using Tekla.Structures.DrawingPresentationPluginInterface;

using TS = Tekla.Structures;
using TSM = Tekla.Structures.Model;
using TSD = Tekla.Structures.Drawing;
using TSP = Tekla.Structures.Plugins;
using TSG = Tekla.Structures.Geometry3d;

namespace CustomPresentationPlugin
{
    using System;
    using System.Collections.Generic;
    using Tekla.Common.Geometry;
    using TS.Solid;
    using TSG;

    [Export(typeof(IDrawingPresentationPlugin))]
    [ExportMetadata("ObjectType", new CustomPresentationObjectTypesEnum[]
    {
        CustomPresentationObjectTypesEnum.Pours,
        CustomPresentationObjectTypesEnum.Parts,
    })]
    [ExportMetadata("BriefDescription", "Bogus Edge Remover")]
    [ExportMetadata("Description", "Presentation plugin that removes the edges of a part that are a result of a non-planar surface.")]
    [ExportMetadata("GUID", "00CE0BCD-429B-48AC-A235-DC14311204D4")]
    public class BogusEdgeRemover : IDrawingPresentationPlugin
    {
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
        private TSD.View.ViewTypes ViewType { get; set; }

        public Segment CreatePresentation(Segment presentation)
        {
            if (presentation == null)
                return null;

            var drawingPart = GetDrawingPart(presentation.Id);
            if (drawingPart == null)
                return presentation;

            if (_model.SelectModelObject(drawingPart.ModelIdentifier) is not TSM.Part modelPart)
                return presentation;

            var view = drawingPart.GetView() as TSD.View;
            if (view == null)
                return presentation;

            ViewType = view.ViewType;
            Scale = view.Attributes.Scale;
            TransformationMatrix = MatrixFactory.ToCoordinateSystem(view.DisplayCoordinateSystem);

            Vector viewAxisX = view.ViewCoordinateSystem.AxisX;
            Vector viewAxisY = view.ViewCoordinateSystem.AxisY;
            Vector viewAxisZ = viewAxisX.Cross(viewAxisY);

            var edgesToDelete = GetModelEdgesInDrawingToBeDeletedInDrawing(modelPart, viewAxisZ);
            if (edgesToDelete.Count > 0)
            {
                RemoveBogusLines(presentation, edgesToDelete);
            }

            return presentation;
        }

        #region Główna logika usuwania linii

        private void RemoveBogusLines(Segment presentation, List<ModelEdgePair> modelEdgesToBeDeleted)
        {
            if (presentation?.Primitives == null || presentation.Primitives.Count == 0)
                return;

            int removedHiddenLinesCount = 0;

            for (int i = 0; i < presentation.Primitives.Count; i++)
            {
                if (presentation.Primitives[i] is PrimitiveGroup group)
                {
                    RemoveBogusLinesInPrimitiveGroup(presentation, modelEdgesToBeDeleted, group, ref removedHiddenLinesCount);
                }
            }
        }

        private void RemoveBogusLinesInPrimitiveGroup(
            Segment presentation,
            List<ModelEdgePair> modelEdgesToBeDeleted,
            PrimitiveGroup primitiveGroup,
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
                        if (!ShouldDeleteLine(presentation, linePrimitive, modelEdgesToBeDeleted, fullLinePrimitiveGroup, ref removedHiddenLinesCount))
                        {
                            newPrimitives.Add(linePrimitive);
                        }
                        break;

                    case PrimitiveGroup nestedGroup:
                        RemoveBogusLinesInPrimitiveGroup(presentation, modelEdgesToBeDeleted, nestedGroup, ref removedHiddenLinesCount);
                        newPrimitives.Add(nestedGroup);
                        break;

                    default:
                        newPrimitives.Add(primitiveBase);
                        break;
                }
            }

            primitiveGroup.Primitives.Clear();
            foreach (var primitive in newPrimitives)
            {
                primitiveGroup.Primitives.Add(primitive);
            }
        }

        private bool ShouldDeleteLine(
            Segment presentation,
            LinePrimitive linePrimitive,
            List<ModelEdgePair> modelEdgesToBeDeleted,
            bool fullLinePrimitiveGroup,
            ref int removedHiddenLinesCount)
        {
            if (!LinePrimitiveShouldBeDeleted(linePrimitive, fullLinePrimitiveGroup, modelEdgesToBeDeleted))
                return false;

            if (!LinePrimitiveIsNotExternal(linePrimitive, presentation))
                return false;

            removedHiddenLinesCount++;
            return true;
        }

        #endregion

        #region Detekcja linii zewnętrznych / wewnętrznych

        private bool LinePrimitiveIsNotExternal(LinePrimitive linePrimitive, Segment presentation)
        {
            if (presentation?.Primitives == null)
                return true;

            LineIntersections top = new(false, false, false);
            LineIntersections bottom = new(false, false, false);
            LineIntersections right = new(false, false, false);
            LineIntersections left = new(false, false, false);

            GetVerticalAndHorizontalCenterLines(linePrimitive, 0.25, out var verticalLine01, out var horizontalLine01, out _);
            GetVerticalAndHorizontalCenterLines(linePrimitive, 0.5, out var verticalLine05, out var horizontalLine05, out var centerPoint);
            GetVerticalAndHorizontalCenterLines(linePrimitive, 0.75, out var verticalLine09, out var horizontalLine09, out _);

            foreach (var primitive in presentation.Primitives)
            {
                if (primitive is PrimitiveGroup primitiveGroup)
                {
                    CheckLinePrimitiveIsNotExternalInPrimitiveGroup(
                        verticalLine01,
                        horizontalLine01,
                        verticalLine05,
                        horizontalLine05,
                        verticalLine09,
                        horizontalLine09,
                        centerPoint,
                        primitiveGroup,
                        ref top,
                        ref bottom,
                        ref right,
                        ref left);
                }
            }

            bool linePrimitiveIsNotExternal =
                   top    is { Line01: true, Line05: true, Line09: true }
                && bottom is { Line01: true, Line05: true, Line09: true }
                && right  is { Line01: true, Line05: true, Line09: true }
                && left   is { Line01: true, Line05: true, Line09: true };

            return linePrimitiveIsNotExternal;
        }

        private void CheckLinePrimitiveIsNotExternalInPrimitiveGroup(
            List<Line> verticalLine01,
            List<Line> horizontalLine01,
            List<Line> verticalLine05,
            List<Line> horizontalLine05,
            List<Line> verticalLine09,
            List<Line> horizontalLine09,
            Point centerPoint,
            PrimitiveGroup primitiveGroup,
            ref LineIntersections top,
            ref LineIntersections bottom,
            ref LineIntersections right,
            ref LineIntersections left)
        {
            if (primitiveGroup?.Primitives == null)
                return;

            foreach (PrimitiveBase primitiveBase in primitiveGroup.Primitives)
            {
                switch (primitiveBase)
                {
                    case LinePrimitive currentLinePrimitive:
                        LineOrArcPrimitiveIsNotExternal(
                            verticalLine01,
                            horizontalLine01,
                            verticalLine05,
                            horizontalLine05,
                            verticalLine09,
                            horizontalLine09,
                            centerPoint,
                            currentLinePrimitive,
                            null,
                            ref top,
                            ref bottom,
                            ref right,
                            ref left);
                        break;

                    case ArcPrimitive currentArcPrimitive:
                        LineOrArcPrimitiveIsNotExternal(
                            verticalLine01,
                            horizontalLine01,
                            verticalLine05,
                            horizontalLine05,
                            verticalLine09,
                            horizontalLine09,
                            centerPoint,
                            null,
                            currentArcPrimitive,
                            ref top,
                            ref bottom,
                            ref right,
                            ref left);
                        break;

                    case PrimitiveGroup nestedGroup:
                        CheckLinePrimitiveIsNotExternalInPrimitiveGroup(
                            verticalLine01,
                            horizontalLine01,
                            verticalLine05,
                            horizontalLine05,
                            verticalLine09,
                            horizontalLine09,
                            centerPoint,
                            nestedGroup,
                            ref top,
                            ref bottom,
                            ref right,
                            ref left);
                        break;
                }
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

            if (currentLineStartPoint == null || currentLineEndPoint == null)
                return;

            Line currentLine = new(currentLineStartPoint, currentLineEndPoint);

            CheckWhereLinesIntersect(
                verticalLine01,
                true,
                centerPoint,
                currentLineStartPoint,
                currentLineEndPoint,
                currentLine,
                ref top.Line01,
                ref bottom.Line01,
                ref right.Line01,
                ref left.Line01);

            CheckWhereLinesIntersect(
                horizontalLine01,
                false,
                centerPoint,
                currentLineStartPoint,
                currentLineEndPoint,
                currentLine,
                ref top.Line01,
                ref bottom.Line01,
                ref right.Line01,
                ref left.Line01);

            CheckWhereLinesIntersect(
                verticalLine05,
                true,
                centerPoint,
                currentLineStartPoint,
                currentLineEndPoint,
                currentLine,
                ref top.Line05,
                ref bottom.Line05,
                ref right.Line05,
                ref left.Line05);

            CheckWhereLinesIntersect(
                horizontalLine05,
                false,
                centerPoint,
                currentLineStartPoint,
                currentLineEndPoint,
                currentLine,
                ref top.Line05,
                ref bottom.Line05,
                ref right.Line05,
                ref left.Line05);

            CheckWhereLinesIntersect(
                verticalLine09,
                true,
                centerPoint,
                currentLineStartPoint,
                currentLineEndPoint,
                currentLine,
                ref top.Line09,
                ref bottom.Line09,
                ref right.Line09,
                ref left.Line09);

            CheckWhereLinesIntersect(
                horizontalLine09,
                false,
                centerPoint,
                currentLineStartPoint,
                currentLineEndPoint,
                currentLine,
                ref top.Line09,
                ref bottom.Line09,
                ref right.Line09,
                ref left.Line09);
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

                if (intersection == null)
                    continue;

                if (intersection.Length() >= DrawingEpsilon)
                    continue;

                if (Distance.PointToPoint(currentLineStartPoint, intersection.Point1) <= DrawingEpsilon)
                    continue;

                if (Distance.PointToPoint(currentLineEndPoint, intersection.Point1) <= DrawingEpsilon)
                    continue;

                if (new Vector(currentLineStartPoint - intersection.Point1)
                        .GetAngleBetween(new Vector(currentLineEndPoint - intersection.Point1)) <= Degrees90)
                    continue;

                if (vertical)
                {
                    if (intersection.Point1.Y > centerPoint.Y + DrawingEpsilon)
                        top = true;
                    else if (intersection.Point1.Y < centerPoint.Y - DrawingEpsilon)
                        bottom = true;
                }
                else
                {
                    if (intersection.Point1.X > centerPoint.X + DrawingEpsilon)
                        right = true;
                    else if (intersection.Point1.X < centerPoint.X - DrawingEpsilon)
                        left = true;
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

                if (modelEdgeToBeDeleted.VisibleLine || fullLinePrimitiveGroup == modelEdgeToBeDeleted.VisibleLine)
                    return true;
            }

            return false;
        }

        private static bool LinePrimitiveOverlapsWithEdgeToBeDeleted(
            LinePrimitive linePrimitive,
            ModelEdgePair modelEdgeToBeDeleted)
        {
            var edge = modelEdgeToBeDeleted.ModelEdgeInDrawing;

            bool endpointsMatch =
                (linePrimitive.StartPoint.DistanceTo(edge.StartPoint) < DrawingEpsilon &&
                 linePrimitive.EndPoint.DistanceTo(edge.EndPoint)   < DrawingEpsilon)
             || (linePrimitive.StartPoint.DistanceTo(edge.EndPoint) < DrawingEpsilon &&
                 linePrimitive.EndPoint.DistanceTo(edge.StartPoint) < DrawingEpsilon);

            if (endpointsMatch)
                return true;

            Line edgeLine = new(
                new Point(edge.StartPoint.X, edge.StartPoint.Y),
                new Point(edge.EndPoint.X,   edge.EndPoint.Y));

            double distanceAllowance = edge.StartPoint.DistanceTo(edge.EndPoint) / 0.9;

            bool startOnLine =
                Distance.PointToLine(new Point(linePrimitive.StartPoint.X, linePrimitive.StartPoint.Y), edgeLine) < DrawingEpsilon &&
                (linePrimitive.StartPoint.DistanceTo(edge.StartPoint) < distanceAllowance ||
                 linePrimitive.StartPoint.DistanceTo(edge.EndPoint)   < distanceAllowance);

            bool endOnLine =
                Distance.PointToLine(new Point(linePrimitive.EndPoint.X, linePrimitive.EndPoint.Y), edgeLine) < DrawingEpsilon &&
                (linePrimitive.EndPoint.DistanceTo(edge.StartPoint) < distanceAllowance ||
                 linePrimitive.EndPoint.DistanceTo(edge.EndPoint)   < distanceAllowance);

            return startOnLine && endOnLine;
        }

        #endregion

        #region Analiza bryły modelu

        private List<ModelEdgePair> GetModelEdgesInDrawingToBeDeletedInDrawing(
            TSM.Part selectedModelPart,
            Vector viewAxisZ)
        {
            var modelEdgesInDrawing = new List<ModelEdgePair>();

            TSM.Solid solid = selectedModelPart.GetSolid();
            FaceEnumerator faceEnum = solid.GetFaceEnumerator();

            while (faceEnum.MoveNext())
            {
                if (faceEnum.Current is not Face currentFace)
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
                    Vector2 startPointInDrawing = new(
                        transformedStartPoint.X / Scale,
                        transformedStartPoint.Y / Scale);

                    Point transformedEndPoint = TransformationMatrix.Transform(commonEdge.EndPoint);
                    Vector2 endPointInDrawing = new(
                        transformedEndPoint.X / Scale,
                        transformedEndPoint.Y / Scale);

                    var commonEdgeInDrawing = new LinePrimitive(startPointInDrawing, endPointInDrawing);

                    Vector middleNormal = new(
                        (currentFace.Normal.X + faceWithSimilarNormal.Normal.X) / 2.0,
                        (currentFace.Normal.Y + faceWithSimilarNormal.Normal.Y) / 2.0,
                        (currentFace.Normal.Z + faceWithSimilarNormal.Normal.Z) / 2.0);

                    bool visibleLine = middleNormal.GetAngleBetween(viewAxisZ) < Degrees90 + SmallAngleAllowance;

                    if (!CommonEdgeIsPresentInModelEdges(commonEdgeInDrawing, visibleLine, modelEdgesInDrawing))
                    {
                        modelEdgesInDrawing.Add(new ModelEdgePair(commonEdgeInDrawing, visibleLine));
                    }
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

        private static bool CommonEdgeIsPresentInModelEdges(
            LinePrimitive commonEdgeInDrawing,
            bool visibleLine,
            List<ModelEdgePair> modelEdgesInDrawing)
        {
            foreach (ModelEdgePair modelEdge in modelEdgesInDrawing)
            {
                bool sameEndPoints =
                    (commonEdgeInDrawing.StartPoint.DistanceTo(modelEdge.ModelEdgeInDrawing.StartPoint) < DrawingEpsilon 
                    && commonEdgeInDrawing.EndPoint.DistanceTo(modelEdge.ModelEdgeInDrawing.EndPoint) < DrawingEpsilon)
                    || (commonEdgeInDrawing.StartPoint.DistanceTo(modelEdge.ModelEdgeInDrawing.EndPoint) < DrawingEpsilon 
                    && commonEdgeInDrawing.EndPoint.DistanceTo(modelEdge.ModelEdgeInDrawing.StartPoint) < DrawingEpsilon);

                if (sameEndPoints && modelEdge.VisibleLine == visibleLine)
                    return true;
            }

            return false;
        }

        private LineSegment GetCommonEdge(Face currentFace, Face faceWithSimilarNormal)
        {
            var commonVertexes = new List<Point>();
            var currentFaceVertexes = GetFaceVertexes(currentFace);
            var similarNormalVertexes = GetFaceVertexes(faceWithSimilarNormal);

            foreach (Point currentVertex in currentFaceVertexes)
            {
                foreach (Point similarNormalVertex in similarNormalVertexes)
                {
                    if (Distance.PointToPoint(currentVertex, similarNormalVertex) < ModelEpsilon)
                    {
                        commonVertexes.Add(currentVertex);
                    }
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
                if (loopEnum.Current is not Loop loop)
                    continue;

                VertexEnumerator vertexEnum = loop.GetVertexEnumerator();
                while (vertexEnum.MoveNext())
                {
                    Point vertex = vertexEnum.Current;
                    if (vertex != null)
                    {
                        faceVertexes.Add(vertex);
                    }
                }
            }

            return faceVertexes;
        }

        private List<Face> GetFacesWithSimilarNormal(Face currentFace, FaceEnumerator faceEnumerator)
        {
            var facesWithSimilarNormal = new List<Face>();

            while (faceEnumerator.MoveNext())
            {
                if (faceEnumerator.Current is not Face secondaryFace)
                    continue;

                if (secondaryFace.Equals(currentFace))
                    continue;

                if (currentFace.Normal.GetAngleBetween(secondaryFace.Normal) < BigAngleAllowance &&
                    FaceIsNotHorizontalNorVertical(secondaryFace))
                {
                    facesWithSimilarNormal.Add(secondaryFace);
                }
            }

            return facesWithSimilarNormal;
        }

        #endregion

        #region Pomocnicze

        private static TSD.Part GetDrawingPart(int drawingId)
        {
            var identifier = new TS.Identifier(drawingId);
            var input = new TSP.DrawingPluginBase.InputDefinition(identifier, identifier);
            return TSD.Tools.InputDefinitionFactory.GetDrawingObject(input) as TSD.Part;
        }

        private struct ModelEdgePair
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
