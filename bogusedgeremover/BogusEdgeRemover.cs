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
    using System.Threading;
    using Tekla.Common.Geometry;
    using TS.Solid;
    using TSG;

    [Export(typeof(IDrawingPresentationPlugin))]
    [ExportMetadata("ObjectType", new CustomPresentationObjectTypesEnum[] {
        CustomPresentationObjectTypesEnum.Pours,
        CustomPresentationObjectTypesEnum.Parts,
    })]
    [ExportMetadata("BriefDescription", "Bogus Edge Remover")]
    [ExportMetadata("Description", "Presentation plugin that removes the edges of a part that are a result of a non-planar surface.")]
    [ExportMetadata("GUID", "00CE0BCD-429B-48AC-A235-DC14311204D4")]
    public class BogusEdgeRemover : IDrawingPresentationPlugin
    {
        /// <summary> The epsilon used for model. </summary>
        private const double ModelEpsilon = 1.0;

        /// <summary> The epsilon used for drawing. </summary>
        private const double DrawingEpsilon = 0.0001;

        /// <summary> 180 degrees expressed in radians. </summary>
        private const double Degrees180 = Math.PI;

        /// <summary> 90 degrees expressed in radians. </summary>
        private const double Degrees90 = Math.PI / 2;

        /// <summary> The allowance used for angles. </summary>
        private const double BigAngleAllowance = Math.PI / 2.5;

        /// <summary> The allowance used for angles. </summary>
        private const double SmallAngleAllowance = Math.PI / 32;

        /// <summary> The epsilon used for angles. </summary>
        private const double AngleEpsilon = 0.001;

        /// <summary> The Z axis of the global coordinate system. </summary>
        private readonly Vector _globalAxisZ = new(0.0, 0.0, 1.0);

        private const int RemovedHiddenLinesCountLimit = 20;

        private readonly TSM.Model _model = new();

        private double Scale { get; set; }

        private Matrix TransformationMatrix { get; set; }

        private TSD.View.ViewTypes ViewType { get; set; }

        public Segment CreatePresentation(Segment presentation)
        {

            if (presentation != null)
            {
                var drawingPart = GetDrawingPart(presentation.Id);

                if (this._model.SelectModelObject(drawingPart.ModelIdentifier) is TSM.Part selectedModelPart)
                {
                    this.ViewType = ((TSD.View)drawingPart.GetView()).ViewType;
                    this.Scale = ((TSD.View)drawingPart.GetView()).Attributes.Scale;
                    this.TransformationMatrix = MatrixFactory.ToCoordinateSystem(((TSD.View)drawingPart.GetView()).DisplayCoordinateSystem);

                    Vector viewAxisX = ((TSD.View)drawingPart.GetView()).ViewCoordinateSystem.AxisX;
                    Vector viewAxisY = ((TSD.View)drawingPart.GetView()).ViewCoordinateSystem.AxisY;
                    Vector viewAxisZ = viewAxisX.Cross(viewAxisY);


                    List<ModelEdgePair> modelEdgesToBeDeleted = this.GetModelEdgesInDrawingToBeDeletedInDrawing(selectedModelPart, viewAxisZ);

                    this.RemoveBogusLines(presentation, modelEdgesToBeDeleted);

                    ////this.DrawModelEdgesToBeDeleted(presentation, modelEdgesToBeDeleted);
                }
            }

            return presentation;
        }

        private void DrawModelEdgesToBeDeleted(Segment presentation, List<ModelEdgePair> modelEdgesToBeDeleted)
        {
            PrimitiveGroup primitiveGroup0 = presentation.Primitives[0] as PrimitiveGroup;
            PrimitiveGroup addedPrimitiveGroup = new PrimitiveGroup(7873, new Pen(2, 1, 3), primitiveGroup0.Brush, primitiveGroup0.GroupType);

            foreach (ModelEdgePair singlePrimitiveBaseCopy in modelEdgesToBeDeleted)
            {
                addedPrimitiveGroup.Primitives.Add(singlePrimitiveBaseCopy.ModelEdgeInDrawing);
            }

            presentation.Primitives.Add(addedPrimitiveGroup);
        }

        private void RemoveBogusLines(Segment presentation, List<ModelEdgePair> modelEdgesToBeDeleted)
        {
            int removedHiddenLinesCount = 0;

            foreach (var primitive in presentation.Primitives) {
                PrimitiveGroup primitiveGroup = primitive as PrimitiveGroup;

                this.RemoveBogusLinesInPrimitiveGroup(presentation, modelEdgesToBeDeleted, ref primitiveGroup, ref removedHiddenLinesCount);
            }
        }

        private void RemoveBogusLinesInPrimitiveGroup(Segment presentation,
            List<ModelEdgePair> modelEdgesToBeDeleted,
            ref PrimitiveGroup primitiveGroup,
            ref int removedHiddenLinesCount)
        {
            List<PrimitiveBase> primitiveBaseCopy = new List<PrimitiveBase>();
            bool fullLinePrimitiveGroup = primitiveGroup.Pen.LineType == 1;

            foreach (PrimitiveBase primitiveBase in primitiveGroup.Primitives)
            {
                switch (primitiveBase) {
                    case LinePrimitive linePrimitive:
                        this.LinePrimitiveShouldBeRemovedOrCopied(
                            presentation,
                            linePrimitive,
                            primitiveBase,
                            modelEdgesToBeDeleted,
                            primitiveBaseCopy,
                            fullLinePrimitiveGroup,
                            ref removedHiddenLinesCount);
                        break;
                    case PrimitiveGroup groupPrimitive:
                        this.RemoveBogusLinesInPrimitiveGroup(presentation, modelEdgesToBeDeleted, ref groupPrimitive, ref removedHiddenLinesCount);
                        primitiveBaseCopy.Add(groupPrimitive);
                        break;
                    default:
                        primitiveBaseCopy.Add(primitiveBase);
                        break;
                }
            }

            primitiveGroup.Primitives.Clear();

            foreach (PrimitiveBase singlePrimitiveBaseCopy in primitiveBaseCopy)
            {
                primitiveGroup.Primitives.Add(singlePrimitiveBaseCopy);
            }
        }

        private void LinePrimitiveShouldBeRemovedOrCopied(
            Segment presentation,
            LinePrimitive linePrimitive,
            PrimitiveBase primitiveBase,
            List<ModelEdgePair> modelEdgesToBeDeleted, 
            List<PrimitiveBase> primitiveBaseCopy, 
            bool fullLinePrimitiveGroup,
            ref int removedHiddenLinesCount)
        {
            if (LinePrimitiveShouldBeDeleted(linePrimitive, fullLinePrimitiveGroup, modelEdgesToBeDeleted)
               && this.LinePrimitiveIsNotExternal(linePrimitive, presentation))
            {
                removedHiddenLinesCount++;
            }
            else
            {
                primitiveBaseCopy.Add(primitiveBase);
            }
        }

        private bool LinePrimitiveIsNotExternal(LinePrimitive linePrimitive, Segment presentation)
        {
            bool linePrimitiveIsNotExternal = true;

            LineIntersections top = new LineIntersections(false, false, false);
            LineIntersections bottom = new LineIntersections(false, false, false);
            LineIntersections right = new LineIntersections(false, false, false);
            LineIntersections left = new LineIntersections(false, false, false);

            GetVerticalAndHorizontalCenterLines(linePrimitive, 0.25, out var verticalLine01, out var horizontalLine01, out _);
            GetVerticalAndHorizontalCenterLines(linePrimitive, 0.5, out var verticalLine05, out var horizontalLine05, out var centerPoint);
            GetVerticalAndHorizontalCenterLines(linePrimitive, 0.75, out var verticalLine09, out var horizontalLine09, out _);

            foreach (var primitive in presentation.Primitives) {
                PrimitiveGroup primitiveGroup = primitive as PrimitiveGroup;

                this.CheckLinePrimitiveIsNotExternalInPrimitiveGroup(
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

            linePrimitiveIsNotExternal = top is { Line01: true, Line05: true, Line09: true }
                                         && bottom is { Line01: true, Line05: true, Line09: true }
                                         && right is { Line01: true, Line05: true, Line09: true }
                                         && left is { Line01: true, Line05: true, Line09: true };

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
            foreach (PrimitiveBase primitiveBase in primitiveGroup.Primitives)
            {
                LinePrimitive currentLinePrimitive = primitiveBase as LinePrimitive;
                ArcPrimitive currentArcPrimitive = primitiveBase as ArcPrimitive;

                if (currentLinePrimitive != null || currentArcPrimitive != null)
                {
                    LineOrArcPrimitiveIsNotExternal(
                        verticalLine01,
                        horizontalLine01,
                        verticalLine05,
                        horizontalLine05,
                        verticalLine09,
                        horizontalLine09,
                        centerPoint,
                        currentLinePrimitive,
                        currentArcPrimitive,
                        ref top,
                        ref bottom,
                        ref right,
                        ref left);
                }
                else if (primitiveBase is PrimitiveGroup groupPrimitive)
                {
                    this.CheckLinePrimitiveIsNotExternalInPrimitiveGroup(
                        verticalLine01,
                        horizontalLine01,
                        verticalLine05,
                        horizontalLine05,
                        verticalLine09,
                        horizontalLine09,
                        centerPoint,
                        groupPrimitive,
                        ref top,
                        ref bottom,
                        ref right,
                        ref left);
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
            Point currentLineStartPoint = null, currentLineEndPoint = null;
            if (currentLinePrimitive != null)
            {
                currentLineStartPoint = new Point(currentLinePrimitive.StartPoint.X, currentLinePrimitive.StartPoint.Y);
                currentLineEndPoint = new Point(currentLinePrimitive.EndPoint.X, currentLinePrimitive.EndPoint.Y);
            }
            else if (currentArcPrimitive != null)
            {
                currentLineStartPoint = new Point(currentArcPrimitive.StartPoint.X, currentArcPrimitive.StartPoint.Y);
                currentLineEndPoint = new Point(currentArcPrimitive.EndPoint.X, currentArcPrimitive.EndPoint.Y);
            }

            if (currentLineStartPoint != null)
            {
                Line currentLine = new Line(currentLineStartPoint, currentLineEndPoint);

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

                if (intersection != null
                    && intersection.Length() < DrawingEpsilon
                    && Distance.PointToPoint(currentLineStartPoint, intersection.Point1) > DrawingEpsilon
                    && Distance.PointToPoint(currentLineEndPoint, intersection.Point1) > DrawingEpsilon
                    && new Vector(currentLineStartPoint - intersection.Point1).GetAngleBetween(new Vector(currentLineEndPoint - intersection.Point1)) > Degrees90)
                {
                    if (vertical)
                    {
                        if (intersection.Point1.Y > centerPoint.Y + DrawingEpsilon)
                        {
                            top = true;
                        }
                        else if (intersection.Point1.Y < centerPoint.Y - DrawingEpsilon)
                        {
                            bottom = true;
                        }
                    }
                    else
                    {
                        if (intersection.Point1.X > centerPoint.X + DrawingEpsilon)
                        {
                            right = true;
                        }
                        else if (intersection.Point1.X < centerPoint.X - DrawingEpsilon)
                        {
                            left = true;
                        }
                    }
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

            centerPoint = new Point((lineRatio * linePrimitive.StartPoint.X +  (1 - lineRatio) * linePrimitive.EndPoint.X), (lineRatio * linePrimitive.StartPoint.Y + (1 - lineRatio) * linePrimitive.EndPoint.Y));
            verticalLine.Add(new Line(centerPoint, new Vector(new Point(0.0, 1.0))));
            verticalLine.Add(new Line(centerPoint, new Vector(new Point(1.0, 3.5))));
            verticalLine.Add(new Line(centerPoint, new Vector(new Point(-1.0, 3.5))));

            horizontalLine.Add(new Line(centerPoint, new Vector(new Point(1.0, 0.0))));
            horizontalLine.Add(new Line(centerPoint, new Vector(new Point(3.5, 1.0))));
            horizontalLine.Add(new Line(centerPoint, new Vector(new Point(3.5, -1.0))));
        }

        private static bool LinePrimitiveShouldBeDeleted(LinePrimitive linePrimitive, bool fullLinePrimitiveGroup, List<ModelEdgePair> modelEdgesToBeDeleted)
        {
            bool linePrimitiveShouldBeDeleted = false;

            foreach (ModelEdgePair modelEdgeToBeDeleted in modelEdgesToBeDeleted)
            {
                if (LinePrimitiveOverlapsWithEdgeToBeDeleted(linePrimitive, modelEdgeToBeDeleted)
                    && (modelEdgeToBeDeleted.VisibleLine || fullLinePrimitiveGroup == modelEdgeToBeDeleted.VisibleLine))
                {
                    linePrimitiveShouldBeDeleted = true;
                    break;
                }
            }

            return linePrimitiveShouldBeDeleted;
        }

        private static bool LinePrimitiveOverlapsWithEdgeToBeDeleted(LinePrimitive linePrimitive, ModelEdgePair modelEdgeToBeDeleted)
        {
            bool linePrimitiveOverlaps = false;

            if ((linePrimitive.StartPoint.DistanceTo(modelEdgeToBeDeleted.ModelEdgeInDrawing.StartPoint) < DrawingEpsilon
                && linePrimitive.EndPoint.DistanceTo(modelEdgeToBeDeleted.ModelEdgeInDrawing.EndPoint) < DrawingEpsilon)
                || (linePrimitive.StartPoint.DistanceTo(modelEdgeToBeDeleted.ModelEdgeInDrawing.EndPoint) < DrawingEpsilon
                && linePrimitive.EndPoint.DistanceTo(modelEdgeToBeDeleted.ModelEdgeInDrawing.StartPoint) < DrawingEpsilon))
            {
                linePrimitiveOverlaps = true;
            }
            else
            {
                Line modelEdgeToBeDeletedLine = new Line(
                    new Point(modelEdgeToBeDeleted.ModelEdgeInDrawing.StartPoint.X, modelEdgeToBeDeleted.ModelEdgeInDrawing.StartPoint.Y),
                    new Point(modelEdgeToBeDeleted.ModelEdgeInDrawing.EndPoint.X, modelEdgeToBeDeleted.ModelEdgeInDrawing.EndPoint.Y));

                double distanceAllowance = modelEdgeToBeDeleted.ModelEdgeInDrawing.StartPoint.DistanceTo(modelEdgeToBeDeleted.ModelEdgeInDrawing.EndPoint) / 0.9;

                if ((Distance.PointToLine(new Point(linePrimitive.StartPoint.X, linePrimitive.StartPoint.Y), modelEdgeToBeDeletedLine) < DrawingEpsilon
                    && (linePrimitive.StartPoint.DistanceTo(modelEdgeToBeDeleted.ModelEdgeInDrawing.StartPoint) < distanceAllowance || linePrimitive.StartPoint.DistanceTo(modelEdgeToBeDeleted.ModelEdgeInDrawing.EndPoint) < distanceAllowance))
                    && (Distance.PointToLine(new Point(linePrimitive.EndPoint.X, linePrimitive.EndPoint.Y), modelEdgeToBeDeletedLine) < DrawingEpsilon
                    && (linePrimitive.EndPoint.DistanceTo(modelEdgeToBeDeleted.ModelEdgeInDrawing.StartPoint) < distanceAllowance || linePrimitive.EndPoint.DistanceTo(modelEdgeToBeDeleted.ModelEdgeInDrawing.EndPoint) < distanceAllowance)))
                {
                    linePrimitiveOverlaps = true;
                }
            }

            return linePrimitiveOverlaps;
        }

        private List<ModelEdgePair> GetModelEdgesInDrawingToBeDeletedInDrawing(TSM.Part selectedModelPart, Vector viewAxisZ)
        {
            List<ModelEdgePair> modelEdgesInDrawing = new List<ModelEdgePair>();

            TSM.Solid solid = selectedModelPart.GetSolid();
            FaceEnumerator myFaceEnum = solid.GetFaceEnumerator();
            while (myFaceEnum.MoveNext())
            {
                if (myFaceEnum.Current is { } currentFace && this.FaceIsNotHorizontalNorVertical(currentFace))
                {
                    List<Face> facesWithSimilarNormal = this.GetFacesWithSimilarNormal(currentFace, solid.GetFaceEnumerator());

                    foreach (Face faceWithSimilarNormal in facesWithSimilarNormal)
                    {
                        LineSegment commonEdge = this.GetCommonEdge(currentFace, faceWithSimilarNormal);

                        if (commonEdge != null)
                        {
                            Point transformedStartPoint = this.TransformationMatrix.Transform(commonEdge.StartPoint);
                            Vector2 startPointInDrawing = new Vector2(transformedStartPoint.X / this.Scale, transformedStartPoint.Y / this.Scale);

                            Point transformedEndPoint = this.TransformationMatrix.Transform(commonEdge.EndPoint);
                            Vector2 endPointInDrawing = new Vector2(transformedEndPoint.X / this.Scale, transformedEndPoint.Y / this.Scale);

                            LinePrimitive commonEdgeInDrawing = new LinePrimitive(startPointInDrawing, endPointInDrawing);
                            Vector middleNormal = new Vector(
                                (currentFace.Normal.X + faceWithSimilarNormal.Normal.X) / 2.0, 
                                (currentFace.Normal.Y + faceWithSimilarNormal.Normal.Y) / 2.0, 
                                (currentFace.Normal.Z + faceWithSimilarNormal.Normal.Z) / 2.0);
                            bool visibleLine = middleNormal.GetAngleBetween(viewAxisZ) < Degrees90 + SmallAngleAllowance;

                            if (!visibleLine)
                            {
                            }

                            if (!CommonEdgeIsPresentInModelEdges(commonEdgeInDrawing, visibleLine, modelEdgesInDrawing))
                            {
                                modelEdgesInDrawing.Add(new ModelEdgePair(commonEdgeInDrawing, visibleLine));                               
                            }
                        }
                    }
                }
            }

            return modelEdgesInDrawing;
        }

        private bool FaceIsNotHorizontalNorVertical(Face currentFace)
        {
            bool faceIsNotHorizontalNorVertical = true;

            double faceAngle = currentFace.Normal.GetAngleBetween(this._globalAxisZ);

            if (faceAngle < AngleEpsilon 
                || (Math.Abs(faceAngle - Degrees90) < AngleEpsilon)
                || faceAngle > Degrees180 - AngleEpsilon)
            {
                faceIsNotHorizontalNorVertical = false;
            }

            return faceIsNotHorizontalNorVertical;
        }

        private static bool CommonEdgeIsPresentInModelEdges(LinePrimitive commonEdgeInDrawing, bool visibleLine, List<ModelEdgePair> modelEdgesInDrawing)
        {
            bool commonEdgeIsPresentInModelEdges = false;

            foreach (ModelEdgePair modelEdge in modelEdgesInDrawing)
            {
                if (((commonEdgeInDrawing.StartPoint.DistanceTo(modelEdge.ModelEdgeInDrawing.StartPoint) < DrawingEpsilon
                        && commonEdgeInDrawing.EndPoint.DistanceTo(modelEdge.ModelEdgeInDrawing.EndPoint) < DrawingEpsilon)
                        || (commonEdgeInDrawing.StartPoint.DistanceTo(modelEdge.ModelEdgeInDrawing.EndPoint) < DrawingEpsilon
                        && commonEdgeInDrawing.EndPoint.DistanceTo(modelEdge.ModelEdgeInDrawing.StartPoint) < DrawingEpsilon))
                        && modelEdge.VisibleLine == visibleLine)
                {
                    commonEdgeIsPresentInModelEdges = true;
                }
            }

            return commonEdgeIsPresentInModelEdges;
        }
            
        private LineSegment GetCommonEdge(Face currentFace, Face faceWithSimilarNormal)
        {
            LineSegment commonEdge = null;
            List<Point> commonVertexes = new List<Point>();
            List<Point> currentFaceVertexes = GetFaceVertexes(currentFace);
            List<Point> similarNormalVertexes = GetFaceVertexes(faceWithSimilarNormal);

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

            if (commonVertexes.Count == 2)
            {
                commonEdge = new LineSegment(commonVertexes[0], commonVertexes[1]);
            }

            return commonEdge;
        }

        private static List<Point> GetFaceVertexes(Face currentFace)
        {
            List<Point> faceVertexes = new List<Point>();
            LoopEnumerator myLoopEnum = currentFace.GetLoopEnumerator();

            while (myLoopEnum.MoveNext())
            {
                if (myLoopEnum.Current is { } myLoop)
                {
                    VertexEnumerator myVertexEnum = myLoop.GetVertexEnumerator();
                    while (myVertexEnum.MoveNext())
                    {
                        Point myVertex = myVertexEnum.Current;
                        if (myVertex != null)
                        {
                            faceVertexes.Add(myVertex);
                        }
                    }
                }
            }

            return faceVertexes;
        }

        private List<Face> GetFacesWithSimilarNormal(Face currentFace, FaceEnumerator faceEnumerator)
        {
            List<Face> facesWithSimilarNormal = new List<Face>();

            while (faceEnumerator.MoveNext())
            {
                if (faceEnumerator.Current is { } secondaryFace && !secondaryFace.Equals(currentFace))
                {
                    if (currentFace.Normal.GetAngleBetween(secondaryFace.Normal) < BigAngleAllowance
                        && this.FaceIsNotHorizontalNorVertical(secondaryFace))
                    {
                        facesWithSimilarNormal.Add(secondaryFace);
                    }
                }
            }

            return facesWithSimilarNormal;
        }

        private static TSD.Part GetDrawingPart(int drawingId)
        {
            var identifier = new TS.Identifier(drawingId);
            var input = new TSP.DrawingPluginBase.InputDefinition(identifier, identifier);
            var drawingPart = TSD.Tools.InputDefinitionFactory.GetDrawingObject(input);

            return drawingPart as TSD.Part;
        }

        /// <summary>Struct to store the assembly pair.</summary>
        private struct ModelEdgePair
        {
            /// <summary>The assembly position.</summary>
            public LinePrimitive ModelEdgeInDrawing;

            /// <summary>The assembly.</summary>
            public readonly bool VisibleLine;

            /// <summary>Initializes a new instance of the <see cref="ModelEdgePair"/> struct.</summary>
            /// <param name="assemblyPosition">The assembly position.</param>
            /// <param name="assembly">The assembly.</param>
            public ModelEdgePair(LinePrimitive modelEdgeInDrawing, bool visibleLine)
            {
                this.ModelEdgeInDrawing = modelEdgeInDrawing;
                this.VisibleLine = visibleLine;
            }
        }

        /// <summary>Struct to store the line intersections.</summary>
        private struct LineIntersections
        {
            /// <summary>The line intersection 01.</summary>
            public bool Line01;

            /// <summary>The line intersection 05.</summary>
            public bool Line05;

            /// <summary>The line intersection 09.</summary>
            public bool Line09;

            /// <summary>Initializes a new instance of the <see cref="LineIntersections"/> struct.</summary>
            /// <param name="line01">The line intersection 01</param>
            /// <param name="line05">The line intersection 05</param>
            /// <param name="line05">The line intersection 09</param>
            public LineIntersections(bool line01, bool line05, bool line09)
            {
                this.Line01 = line01;
                this.Line05 = line05;
                this.Line09 = line09;
            }
        }
    }
}
