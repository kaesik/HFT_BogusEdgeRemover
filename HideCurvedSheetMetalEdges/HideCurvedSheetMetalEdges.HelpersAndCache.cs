using TS = Tekla.Structures;
using TSD = Tekla.Structures.Drawing;
using TSM = Tekla.Structures.Model;
using TSP = Tekla.Structures.Plugins;
using TSDT = Tekla.Structures.Drawing.Tools;

namespace HideCurvedSheetMetalEdges
{
    using System;
    using System.Collections.Generic;

    using Tekla.Common.Geometry;
    using TS.DrawingPresentationModel;
    using TS.Geometry3d;

    public partial class HideCurvedSheetMetalEdges
    {
        #region Pomocnicze – dopasowanie / cache linii modelu

        private const int MaxSolidAnalysisCacheEntries = 256;

        private static readonly object SolidAnalysisCacheLock = new();

        private static readonly Dictionary<SolidAnalysisCacheKey, SolidAnalysisCacheEntry>
            SolidAnalysisCache = new();

        private const int MaxCurvedViewCacheEntries = 64;

        private static readonly object CurvedViewCacheLock = new();

        private static readonly List<CurvedViewCacheItem> CurvedViewCache = new();

        private static bool PresentationContainsLine(Segment presentation)
        {
            return presentation?.Primitives != null && PrimitiveListContainsLine(presentation.Primitives);
        }

        private static bool PrimitiveListContainsLine(IList<PrimitiveBase> primitives)
        {
            if (primitives == null || primitives.Count == 0)
                return false;

            foreach (var primitive in primitives)
            {
                switch (primitive)
                {
                    case LinePrimitive:
                        return true;

                    case PrimitiveGroup group when PrimitiveListContainsLine(group.Primitives):
                        return true;
                }
            }

            return false;
        }

        private static bool ModelEdgeIsPresentInList(
            LinePrimitive edge,
            List<LinePrimitive> list)
        {
            foreach (var e in list)
            {
                bool sameEndPoints =
                    (edge.StartPoint.DistanceTo(e.StartPoint) < DrawingEpsilon &&
                     edge.EndPoint.DistanceTo(e.EndPoint)     < DrawingEpsilon)
                    || (edge.StartPoint.DistanceTo(e.EndPoint) < DrawingEpsilon &&
                        edge.EndPoint.DistanceTo(e.StartPoint) < DrawingEpsilon);

                if (sameEndPoints)
                    return true;
            }

            return false;
        }

        private static void SplitLinesInPrimitiveList(
            IList<PrimitiveBase> primitives,
            List<CachedLine> cachedLines,
            List<PrimitiveBase> output)
        {
            if (primitives == null || primitives.Count == 0)
                return;

            foreach (var primitive in primitives)
            {
                switch (primitive)
                {
                    case LinePrimitive linePrimitive:
                    {
                        var splitLines = SplitLinePrimitiveByIntersections(linePrimitive, cachedLines);

                        foreach (var splitLine in splitLines)
                            output.Add(splitLine);

                        break;
                    }

                    case PrimitiveGroup group:
                    {
                        var newGroupPrimitives = new List<PrimitiveBase>(group.Primitives.Count);

                        SplitLinesInPrimitiveList(group.Primitives, cachedLines, newGroupPrimitives);

                        group.Primitives.Clear();
                        foreach (var p in newGroupPrimitives)
                            group.Primitives.Add(p);

                        output.Add(group);
                        break;
                    }

                    default:
                        output.Add(primitive);
                        break;
                }
            }
        }

        private static TSD.Part GetDrawingPart(int drawingId)
        {
            var identifier = new TS.Identifier(drawingId);
            var input = new TSP.DrawingPluginBase.InputDefinition(identifier, identifier);
            return TSD.Tools.InputDefinitionFactory.GetDrawingObject(input) as TSD.Part;
        }

        private void ConfigureViewProjection(TSD.View view)
        {
            this.CurrentView = view;
            this.Scale = view.Attributes.Scale;

            if (this.Scale <= 0.0)
                this.Scale = 1.0;

            this.TransformationMatrix =
                MatrixFactory.ToCoordinateSystem(view.DisplayCoordinateSystem);

            this.IsUnfolded = view.Attributes is { UnfoldedView: true };
            this.IsCurvedSectionView = false;
            this.CurvedSourceView = null;
            this.CurvedSourceTransformationMatrix = null;

            if (!TryGetCurvedSectionSourceView(view, out TSD.View sourceView))
                return;

            this.IsCurvedSectionView = true;
            this.CurvedSourceView = sourceView;
            this.CurvedSourceTransformationMatrix =
                MatrixFactory.ToCoordinateSystem(sourceView.DisplayCoordinateSystem);
        }

        private static bool TryGetCurvedSectionSourceView(
            TSD.View targetView,
            out TSD.View sourceView)
        {
            sourceView = null;

            if (targetView == null)
                return false;

            lock (CurvedViewCacheLock)
            {
                foreach (CurvedViewCacheItem item in CurvedViewCache)
                {
                    try
                    {
                        if (item.TargetView != null &&
                            item.TargetView.IsSameDatabaseObject(targetView))
                        {
                            sourceView = item.Entry.SourceView;
                            return item.Entry.IsCurvedSectionView && sourceView != null;
                        }
                    }
                    catch
                    {
                        // Obiekt widoku mógł zostać usunięty lub unieważniony.
                    }
                }
            }

            bool isCurvedSectionView =
                TryResolveCurvedSectionSourceView(targetView, out sourceView);

            lock (CurvedViewCacheLock)
            {
                if (CurvedViewCache.Count >= MaxCurvedViewCacheEntries)
                    CurvedViewCache.Clear();

                CurvedViewCache.Add(new CurvedViewCacheItem(
                    targetView,
                    new CurvedViewCacheEntry(isCurvedSectionView, sourceView)));
            }

            return isCurvedSectionView;
        }

        private static bool TryResolveCurvedSectionSourceView(
            TSD.View targetView,
            out TSD.View sourceView)
        {
            sourceView = null;

            try
            {
                var directlyRelatedMarks = targetView.GetRelatedObjects(
                    new[] { typeof(TSD.CurvedSectionMark) });

                while (directlyRelatedMarks.MoveNext())
                {
                    if (directlyRelatedMarks.Current is not TSD.CurvedSectionMark mark)
                        continue;

                    if (mark.GetView() is TSD.View markView &&
                        !markView.IsSameDatabaseObject(targetView))
                    {
                        sourceView = markView;
                        return true;
                    }
                }
            }
            catch
            {
                // Nie wszystkie wersje Tekli zwracają znacznik po stronie widoku wynikowego.
            }

            try
            {
                TSD.Drawing drawing = targetView.GetDrawing();
                TSD.ContainerView sheet = drawing?.GetSheet();
                if (sheet == null)
                    return false;

                var marks = sheet.GetAllObjects(
                    typeof(TSD.CurvedSectionMark));

                while (marks.MoveNext())
                {
                    if (marks.Current is not TSD.CurvedSectionMark mark)
                        continue;

                    if (!CurvedSectionMarkIsRelatedToView(mark, targetView))
                        continue;

                    if (mark.GetView() is not TSD.View markSourceView ||
                        markSourceView.IsSameDatabaseObject(targetView))
                    {
                        continue;
                    }

                    sourceView = markSourceView;
                    return true;
                }
            }
            catch
            {
                return false;
            }

            return false;
        }

        private static bool CurvedSectionMarkIsRelatedToView(
            TSD.CurvedSectionMark mark,
            TSD.View targetView)
        {
            try
            {
                var relatedViews = mark.GetRelatedObjects(
                    new[] { typeof(TSD.View) });

                while (relatedViews.MoveNext())
                {
                    if (relatedViews.Current is TSD.View relatedView &&
                        relatedView.IsSameDatabaseObject(targetView))
                    {
                        return true;
                    }
                }
            }
            catch
            {
                return false;
            }

            return false;
        }

        private List<LinePrimitive> ProjectModelEdgeToPresentation(
            LineSegment modelEdge)
        {
            var result = new List<LinePrimitive>(1);

            if (modelEdge == null)
                return result;

            if (!TryProjectModelPointToPresentation(modelEdge.Point1, out Point start) ||
                !TryProjectModelPointToPresentation(modelEdge.Point2, out Point end))
            {
                return result;
            }

            if (Distance.PointToPoint(start, end) <= DrawingEpsilon)
                return result;

            result.Add(new LinePrimitive(
                new Vector2(start.X, start.Y),
                new Vector2(end.X, end.Y)));

            return result;
        }

        private bool TryProjectModelPointToPresentation(
            Point modelPoint,
            out Point presentationPoint)
        {
            presentationPoint = null;

            if (modelPoint == null)
                return false;

            try
            {
                Point pointInView;

                if (this.IsCurvedSectionView &&
                    this.CurvedSourceView != null &&
                    this.CurrentView != null &&
                    this.CurvedSourceTransformationMatrix != null)
                {
                    Point pointInSourceView =
                        this.CurvedSourceTransformationMatrix.Transform(modelPoint);

                    pointInView = TSDT.DrawingCoordinateConverter.Convert(
                        this.CurvedSourceView,
                        this.CurrentView,
                        pointInSourceView);
                }
                else
                {
                    pointInView = this.TransformationMatrix.Transform(modelPoint);
                }

                if (pointInView == null ||
                    !IsFinite(pointInView.X) ||
                    !IsFinite(pointInView.Y))
                {
                    return false;
                }

                presentationPoint = new Point(
                    pointInView.X / this.Scale,
                    pointInView.Y / this.Scale,
                    0.0);

                return true;
            }
            catch
            {
                // Awaryjnie zachowujemy dotychczasową transformację płaską.
                try
                {
                    Point fallback = this.TransformationMatrix.Transform(modelPoint);
                    presentationPoint = new Point(
                        fallback.X / this.Scale,
                        fallback.Y / this.Scale,
                        0.0);

                    return IsFinite(presentationPoint.X) &&
                           IsFinite(presentationPoint.Y);
                }
                catch
                {
                    presentationPoint = null;
                    return false;
                }
            }
        }

        private static bool IsFinite(double value)
        {
            return !double.IsNaN(value) && !double.IsInfinity(value);
        }

        private static bool SegmentBoundingBoxesOverlap(LineSegment a, LineSegment b, double tolerance)
        {
            double aMinX = Math.Min(a.Point1.X, a.Point2.X) - tolerance;
            double aMaxX = Math.Max(a.Point1.X, a.Point2.X) + tolerance;
            double aMinY = Math.Min(a.Point1.Y, a.Point2.Y) - tolerance;
            double aMaxY = Math.Max(a.Point1.Y, a.Point2.Y) + tolerance;

            double bMinX = Math.Min(b.Point1.X, b.Point2.X) - tolerance;
            double bMaxX = Math.Max(b.Point1.X, b.Point2.X) + tolerance;
            double bMinY = Math.Min(b.Point1.Y, b.Point2.Y) - tolerance;
            double bMaxY = Math.Max(b.Point1.Y, b.Point2.Y) + tolerance;

            return aMinX <= bMaxX && aMaxX >= bMinX &&
                   aMinY <= bMaxY && aMaxY >= bMinY;
        }

        private static bool LinePrimitiveBoundingBoxesOverlap(
            LinePrimitive a,
            LinePrimitive b,
            double tolerance)
        {
            double aMinX = Math.Min(a.StartPoint.X, a.EndPoint.X) - tolerance;
            double aMaxX = Math.Max(a.StartPoint.X, a.EndPoint.X) + tolerance;
            double aMinY = Math.Min(a.StartPoint.Y, a.EndPoint.Y) - tolerance;
            double aMaxY = Math.Max(a.StartPoint.Y, a.EndPoint.Y) + tolerance;

            double bMinX = Math.Min(b.StartPoint.X, b.EndPoint.X) - tolerance;
            double bMaxX = Math.Max(b.StartPoint.X, b.EndPoint.X) + tolerance;
            double bMinY = Math.Min(b.StartPoint.Y, b.EndPoint.Y) - tolerance;
            double bMaxY = Math.Max(b.StartPoint.Y, b.EndPoint.Y) + tolerance;

            return aMinX <= bMaxX && aMaxX >= bMinX &&
                   aMinY <= bMaxY && aMaxY >= bMinY;
        }

        private static List<CachedLine> BuildCachedLines(Segment presentation)
        {
            var cachedLines = new List<CachedLine>();

            if (presentation?.Primitives == null)
                return cachedLines;

            foreach (var primitive in presentation.Primitives)
            {
                switch (primitive)
                {
                    case LinePrimitive lp:
                        cachedLines.Add(new CachedLine(lp));
                        break;

                    case PrimitiveGroup g:
                        CollectAllLinesFromGroup(g, cachedLines);
                        break;
                }
            }

            return cachedLines;
        }

        private static void CollectAllLinesFromGroup(PrimitiveGroup group, List<CachedLine> output)
        {
            if (group?.Primitives == null)
                return;

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

        private SolidAnalysisCacheEntry GetOrBuildSolidAnalysisCache(TSM.Part part)
        {
            string modelPath = _model.GetInfo().ModelPath ?? string.Empty;

            var key = new SolidAnalysisCacheKey(
                modelPath,
                part.Identifier.ID,
                part.ModificationTime?.Ticks ?? 0L);

            lock (SolidAnalysisCacheLock)
            {
                if (SolidAnalysisCache.TryGetValue(key, out var cachedEntry))
                    return cachedEntry;
            }

            SolidAnalysisCacheEntry newEntry = BuildSolidAnalysisCacheEntry(part);

            lock (SolidAnalysisCacheLock)
            {
                if (SolidAnalysisCache.TryGetValue(key, out var existingEntry))
                    return existingEntry;

                RemoveObsoleteCacheEntriesForPart(key.ModelPath, key.PartId);

                if (SolidAnalysisCache.Count >= MaxSolidAnalysisCacheEntries)
                    SolidAnalysisCache.Clear();

                SolidAnalysisCache[key] = newEntry;
                return newEntry;
            }
        }

        private static void RemoveObsoleteCacheEntriesForPart(string modelPath, int partId)
        {
            var keysToRemove = new List<SolidAnalysisCacheKey>();

            foreach (var key in SolidAnalysisCache.Keys)
            {
                if (key.PartId == partId &&
                    string.Equals(key.ModelPath, modelPath, StringComparison.OrdinalIgnoreCase))
                {
                    keysToRemove.Add(key);
                }
            }

            foreach (var key in keysToRemove)
                SolidAnalysisCache.Remove(key);
        }

        private readonly struct CurvedViewCacheItem
        {
            public readonly TSD.View TargetView;
            public readonly CurvedViewCacheEntry Entry;

            public CurvedViewCacheItem(
                TSD.View targetView,
                CurvedViewCacheEntry entry)
            {
                TargetView = targetView;
                Entry = entry;
            }
        }

        private readonly struct CurvedViewCacheEntry
        {
            public readonly bool IsCurvedSectionView;
            public readonly TSD.View SourceView;

            public CurvedViewCacheEntry(
                bool isCurvedSectionView,
                TSD.View sourceView)
            {
                IsCurvedSectionView = isCurvedSectionView;
                SourceView = sourceView;
            }
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
                    new Point(primitive.EndPoint.X,   primitive.EndPoint.Y,   0));
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

        private readonly struct SolidAnalysisCacheKey : IEquatable<SolidAnalysisCacheKey>
        {
            public readonly string ModelPath;
            public readonly int PartId;
            public readonly long ModificationTicks;

            public SolidAnalysisCacheKey(
                string modelPath,
                int partId,
                long modificationTicks)
            {
                ModelPath = modelPath ?? string.Empty;
                PartId = partId;
                ModificationTicks = modificationTicks;
            }

            public bool Equals(SolidAnalysisCacheKey other)
            {
                return PartId == other.PartId &&
                       ModificationTicks == other.ModificationTicks &&
                       string.Equals(
                           ModelPath,
                           other.ModelPath,
                           StringComparison.OrdinalIgnoreCase);
            }

            public override bool Equals(object obj)
            {
                return obj is SolidAnalysisCacheKey other && Equals(other);
            }

            public override int GetHashCode()
            {
                unchecked
                {
                    int modelPathHash = StringComparer.OrdinalIgnoreCase.GetHashCode(ModelPath);
                    int hashCode = modelPathHash;
                    hashCode = (hashCode * 397) ^ PartId;
                    hashCode = (hashCode * 397) ^ ModificationTicks.GetHashCode();
                    return hashCode;
                }
            }
        }

        private sealed class SolidAnalysisCacheEntry
        {
            public readonly List<ModelEdgeCandidate> CandidateEdgesToDelete;
            public readonly List<LineSegment> AllModelEdges;

            public SolidAnalysisCacheEntry(
                List<ModelEdgeCandidate> candidateEdgesToDelete,
                List<LineSegment> allModelEdges)
            {
                CandidateEdgesToDelete = candidateEdgesToDelete;
                AllModelEdges = allModelEdges;
            }
        }

        private readonly struct ModelEdgeCandidate
        {
            public readonly LineSegment Edge;
            public readonly Vector FirstFaceNormal;
            public readonly Vector SecondFaceNormal;
            public readonly Vector MiddleNormal;

            public ModelEdgeCandidate(
                LineSegment edge,
                Vector firstFaceNormal,
                Vector secondFaceNormal,
                Vector middleNormal)
            {
                Edge = edge;
                FirstFaceNormal = firstFaceNormal;
                SecondFaceNormal = secondFaceNormal;
                MiddleNormal = middleNormal;
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
