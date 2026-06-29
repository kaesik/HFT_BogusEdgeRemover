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

        private const double ModelEpsilon        = 1.0;
        private const double DrawingEpsilon      = 0.0001;
        private const double Degrees90           = Math.PI / 2;
        private const double BigAngleAllowance   = Math.PI / 16;
        private const double SmallAngleAllowance = Math.PI / 32;

        private readonly TSM.Model _model = new();

        private double Scale { get; set; }
        private bool IsUnfolded { get; set; }
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

            this.IsUnfolded = view.Attributes is { UnfoldedView: true };

            List<ModelEdgePair> edgesToDelete =
                GetModelEdgesInDrawingToBeDeletedInDrawing(modelPart, viewAxisZ);

            List<LinePrimitive> modelEdgesToKeep =
                GetModelEdgesInDrawingToKeep(modelPart);

            RemoveBogusLines(presentation, edgesToDelete, modelEdgesToKeep);

            return presentation;
        }

        #endregion
    }
}
