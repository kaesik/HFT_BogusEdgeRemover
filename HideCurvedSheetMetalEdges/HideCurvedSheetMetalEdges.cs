using TS = Tekla.Structures;
using TSM = Tekla.Structures.Model;
using TSD = Tekla.Structures.Drawing;

namespace HideCurvedSheetMetalEdges
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel.Composition;

    using TS.DrawingPresentationModel;
    using TS.DrawingPresentationPluginInterface;
    using TS.Geometry3d;

    #region Definicje eksportu wtyczki

    [Export(typeof(IDrawingPresentationPlugin))]
    [ExportMetadata("ObjectType", new[]
    {
        CustomPresentationObjectTypesEnum.Pours,
        CustomPresentationObjectTypesEnum.Parts
    })]
    [ExportMetadata("BriefDescription", "Kanten von gebogenen Blechen ausblenden")]
    [ExportMetadata("Description", "Kanten von gebogenen Blechen werden ausgeblendet.")]
    [ExportMetadata("GUID", "00CE0BCD-429B-48AC-A235-DC14311204D4")]   
    #endregion   
    public partial class HideCurvedSheetMetalEdges : IDrawingPresentationPlugin
    {
        #region Stałe / pola

        private const double ModelEpsilon                  = 1.0;
        private const double DrawingEpsilon                = 0.0001;
        private const double NormalDrawingEpsilonMinimum   = 0.01;
        private const double CurvedDrawingEpsilonMinimum   = 0.05;
        private const double CurvedProjectionSampleLength  = 25.0;
        private const int CurvedProjectionMinSegmentCount  = 4;
        private const int CurvedProjectionMaxSegmentCount  = 96;
        private const double Degrees90                     = Math.PI / 2;
        private const double BigAngleAllowance             = Math.PI / 12;
        private const double SmallAngleAllowance           = Math.PI / 32;

        private readonly TSM.Model _model = new();

        private double Scale { get; set; }
        private bool IsUnfolded { get; set; }
        private bool IsCurvedSectionView { get; set; }
        private Matrix TransformationMatrix { get; set; }
        private Matrix CurvedSourceTransformationMatrix { get; set; }
        private TSD.View CurrentView { get; set; }
        private TSD.View CurvedSourceView { get; set; }

        private bool UseTopologyPipeline =>
            this.IsUnfolded || this.IsCurvedSectionView;

        private double ActiveDrawingEpsilon
        {
            get
            {
                double scaleAdjustedTolerance =
                    ModelEpsilon / Math.Max(this.Scale, 1.0);

                if (!this.IsCurvedSectionView)
                {
                    return Math.Max(
                        NormalDrawingEpsilonMinimum,
                        scaleAdjustedTolerance * 0.1);
                }

                return Math.Max(
                    CurvedDrawingEpsilonMinimum,
                    scaleAdjustedTolerance);
            }
        }

        #endregion

        #region Entry point

        public Segment CreatePresentation(Segment presentation)
        {
            if (presentation == null)
                return null;

            if (!PresentationContainsLine(presentation))
                return presentation;

            var drawingPart = GetDrawingPart(presentation.Id);
            if (drawingPart == null)
                return presentation;

            if (_model.SelectModelObject(drawingPart.ModelIdentifier) is not TSM.Part modelPart)
                return presentation;

            if (drawingPart.GetView() is not TSD.View view)
                return presentation;

            ConfigureViewProjection(view);

            TSD.View directionView = this.IsCurvedSectionView && this.CurvedSourceView != null
                ? this.CurvedSourceView
                : view;

            Vector viewAxisX = directionView.ViewCoordinateSystem.AxisX;
            Vector viewAxisY = directionView.ViewCoordinateSystem.AxisY;
            Vector viewAxisZ = viewAxisX.Cross(viewAxisY);

            List<ModelEdgePair> edgesToDelete =
                GetModelEdgesInDrawingToBeDeletedInDrawing(
                    modelPart,
                    viewAxisZ,
                    this.IsCurvedSectionView);

            List<LinePrimitive> modelEdgesToKeep = this.IsUnfolded
                ? GetModelEdgesInDrawingToKeep(modelPart)
                : new List<LinePrimitive>();

            RemoveBogusLines(presentation, edgesToDelete, modelEdgesToKeep);

            return presentation;
        }

        #endregion
    }
}
