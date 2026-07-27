using TS = Tekla.Structures;

namespace HideCurvedSheetMetalEdges
{
    using System.Collections.Generic;

    using TS.DrawingPresentationModel;

    public partial class HideCurvedSheetMetalEdges
    {
        #region Główna logika – pipeline usuwania linii

        private void RemoveBogusLines(
            Segment presentation,
            List<ModelEdgePair> modelEdgesToBeDeleted,
            List<LinePrimitive> modelEdgesToKeep)
        {
            if (presentation?.Primitives == null || presentation.Primitives.Count == 0)
                return;

            if (this.UseTopologyPipeline)
            {
                var cachedLinesOriginal = BuildCachedLines(presentation);

                var splitRootPrimitives = new List<PrimitiveBase>(presentation.Primitives.Count);
                SplitLinesInPrimitiveList(presentation.Primitives, cachedLinesOriginal, splitRootPrimitives);

                presentation.Primitives.Clear();
                foreach (var p in splitRootPrimitives)
                    presentation.Primitives.Add(p);

                var cachedLines = BuildCachedLines(presentation);
                var graph = BuildLineGraph(cachedLines);

                int removedHiddenLinesCount = 0;
                var newRootPrimitives = new List<PrimitiveBase>(presentation.Primitives.Count);

                foreach (var primitive in presentation.Primitives)
                {
                    switch (primitive)
                    {
                        case LinePrimitive linePrimitive:
                        {
                            if (!ShouldDeleteLine(
                                    linePrimitive,
                                    modelEdgesToBeDeleted,
                                    modelEdgesToKeep,
                                    cachedLines,
                                    graph,
                                    ref removedHiddenLinesCount))
                            {
                                newRootPrimitives.Add(linePrimitive);
                            }

                            break;
                        }

                        case PrimitiveGroup group:
                        {
                            RemoveBogusLinesInPrimitiveGroup(
                                modelEdgesToBeDeleted,
                                modelEdgesToKeep,
                                group,
                                cachedLines,
                                graph,
                                ref removedHiddenLinesCount);

                            newRootPrimitives.Add(group);
                            break;
                        }

                        default:
                            newRootPrimitives.Add(primitive);
                            break;
                    }
                }

                presentation.Primitives.Clear();
                foreach (var p in newRootPrimitives)
                    presentation.Primitives.Add(p);

                return;
            }

            var cachedLinesNonUnfolded = BuildCachedLines(presentation);

            int removedHiddenLinesCountNonUnfolded = 0;
            var newRootPrimitivesNonUnfolded = new List<PrimitiveBase>(presentation.Primitives.Count);

            foreach (var primitive in presentation.Primitives)
            {
                switch (primitive)
                {
                    case LinePrimitive linePrimitive:
                    {
                        if (!LinePrimitiveCanOverlapAnyEdgeToBeDeleted(linePrimitive, modelEdgesToBeDeleted))
                        {
                            newRootPrimitivesNonUnfolded.Add(linePrimitive);
                            break;
                        }

                        var splitLines =
                            SplitLinePrimitiveByIntersectionsAndModelEdges(
                                linePrimitive,
                                cachedLinesNonUnfolded,
                                modelEdgesToBeDeleted);

                        foreach (var splitLine in splitLines)
                        {
                            if (!ShouldDeleteLine(
                                    splitLine,
                                    modelEdgesToBeDeleted,
                                    modelEdgesToKeep,
                                    cachedLinesNonUnfolded,
                                    null,
                                    ref removedHiddenLinesCountNonUnfolded))
                            {
                                newRootPrimitivesNonUnfolded.Add(splitLine);
                            }
                        }

                        break;
                    }

                    case PrimitiveGroup group:
                    {
                        RemoveBogusLinesInPrimitiveGroup(
                            modelEdgesToBeDeleted,
                            modelEdgesToKeep,
                            group,
                            cachedLinesNonUnfolded,
                            null,
                            ref removedHiddenLinesCountNonUnfolded);

                        newRootPrimitivesNonUnfolded.Add(group);
                        break;
                    }

                    default:
                        newRootPrimitivesNonUnfolded.Add(primitive);
                        break;
                }
            }

            presentation.Primitives.Clear();
            foreach (var p in newRootPrimitivesNonUnfolded)
                presentation.Primitives.Add(p);
        }

        private void RemoveBogusLinesInPrimitiveGroup(
            List<ModelEdgePair> modelEdgesToBeDeleted,
            List<LinePrimitive> modelEdgesToKeep,
            PrimitiveGroup primitiveGroup,
            List<CachedLine> cachedLines,
            LineGraph graph,
            ref int removedHiddenLinesCount)
        {
            if (primitiveGroup?.Primitives == null || primitiveGroup.Primitives.Count == 0)
                return;

            var newPrimitives = new List<PrimitiveBase>(primitiveGroup.Primitives.Count);

            foreach (PrimitiveBase primitiveBase in primitiveGroup.Primitives)
            {
                switch (primitiveBase)
                {
                    case LinePrimitive linePrimitive:
                    {
                        if (this.UseTopologyPipeline)
                        {
                            if (!ShouldDeleteLine(
                                    linePrimitive,
                                    modelEdgesToBeDeleted,
                                    modelEdgesToKeep,
                                    cachedLines,
                                    graph,
                                    ref removedHiddenLinesCount))
                            {
                                newPrimitives.Add(linePrimitive);
                            }
                        }
                        else
                        {
                            if (!LinePrimitiveCanOverlapAnyEdgeToBeDeleted(linePrimitive, modelEdgesToBeDeleted))
                            {
                                newPrimitives.Add(linePrimitive);
                                break;
                            }

                            var splitLines =
                                SplitLinePrimitiveByIntersectionsAndModelEdges(
                                    linePrimitive,
                                    cachedLines,
                                    modelEdgesToBeDeleted);

                            foreach (var splitLine in splitLines)
                            {
                                if (!ShouldDeleteLine(
                                        splitLine,
                                        modelEdgesToBeDeleted,
                                        modelEdgesToKeep,
                                        cachedLines,
                                        graph,
                                        ref removedHiddenLinesCount))
                                {
                                    newPrimitives.Add(splitLine);
                                }
                            }
                        }

                        break;
                    }

                    case PrimitiveGroup nestedGroup:
                    {
                        RemoveBogusLinesInPrimitiveGroup(
                            modelEdgesToBeDeleted,
                            modelEdgesToKeep,
                            nestedGroup,
                            cachedLines,
                            graph,
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
            List<LinePrimitive> modelEdgesToKeep,
            List<CachedLine> cachedLines,
            LineGraph graph,
            ref int removedHiddenLinesCount)
        {
            if (this.UseTopologyPipeline)
            {
                bool overlapsDeleteEdge = false;
                if (modelEdgesToBeDeleted is { Count: > 0 })
                {
                    foreach (var modelEdge in modelEdgesToBeDeleted)
                    {
                        if (!LinePrimitiveOverlapsWithEdgeToBeDeleted(linePrimitive, modelEdge))
                            continue;

                        overlapsDeleteEdge = true;
                        break;
                    }
                }

                bool overlapsKeepEdge = false;
                if (modelEdgesToKeep is { Count: > 0 })
                {
                    foreach (var keepEdge in modelEdgesToKeep)
                    {
                        if (!LinePrimitiveOverlapsWithModelEdge(linePrimitive, keepEdge))
                            continue;

                        overlapsKeepEdge = true;
                        break;
                    }
                }

                bool? isInternal = null;

                if (overlapsDeleteEdge)
                {
                    // W curved section view krawędź modelowa jest już jednoznacznym
                    // kandydatem do usunięcia. Dodatkowa klasyfikacja konturu jest
                    // zawodna, gdy Tekla dzieli kontur na krótkie odcinki lub łuki.
                    if (this.IsCurvedSectionView)
                    {
                        removedHiddenLinesCount++;
                        return true;
                    }

                    isInternal = LinePrimitiveIsNotExternal(linePrimitive, cachedLines);
                    if (isInternal.Value)
                    {
                        removedHiddenLinesCount++;
                        return true;
                    }
                }

                if (overlapsKeepEdge)
                    return false;

                if (!isInternal.HasValue)
                    isInternal = LinePrimitiveIsNotExternal(linePrimitive, cachedLines);

                if (!isInternal.Value)
                    return false;

                if (graph != null && IsDiagonalInGraph(linePrimitive, graph))
                {
                    removedHiddenLinesCount++;
                    return true;
                }

                return false;
            }

            if (modelEdgesToBeDeleted == null || modelEdgesToBeDeleted.Count == 0)
                return false;

            foreach (var modelEdge in modelEdgesToBeDeleted)
            {
                if (!LinePrimitiveOverlapsWithEdgeToBeDeleted(
                        linePrimitive,
                        modelEdge))
                {
                    continue;
                }

                // W zwykłym widoku kandydat został już zweryfikowany na poziomie
                // bryły: jest wspólną krawędzią dwóch podobnie skierowanych ścian,
                // znajdujących się po tej samej stronie kierunku patrzenia.
                // Nie jest więc krawędzią sylwetki. Dodatkowa klasyfikacja konturu
                // powodowała pozostawianie poprawnie dopasowanych linii.
                removedHiddenLinesCount++;
                return true;
            }

            return false;
        }

        #endregion
    }
}
