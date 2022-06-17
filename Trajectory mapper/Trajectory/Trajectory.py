 import os
from pyexpat import model
import unittest
import logging
import vtk, qt, ctk, slicer
import math
import numpy
from slicer.ScriptedLoadableModule import *
from slicer.util import VTKObservationMixin

#
# Trajectory
#

class Trajectory(ScriptedLoadableModule):

  def __init__(self, parent):
    ScriptedLoadableModule.__init__(self, parent)
    self.parent.title = "Trajectory"  
    self.parent.categories = ["Informatics"]  
    self.parent.dependencies = ["CurveMaker"]  
    self.parent.contributors = ["Xyz (Alpha Corp.)"]  
    self.parent.helpText = """
    This extension helps us to mark trajectory for needle incision.
    See more information in <a href="https://github.com/ajaykumarsahu0505/Slicer3D/tree/main/Trajectory%20mapper">module documentation</a>."""
    
    self.parent.acknowledgementText = """
    
    """

#
# TrajectoryWidget
#

class TrajectoryWidget(ScriptedLoadableModuleWidget, VTKObservationMixin):

  def __init__(self, parent=None):
    ScriptedLoadableModuleWidget.__init__(self, parent)
    VTKObservationMixin.__init__(self)  
    self.logic = None
    self._parameterNode = None
    self._updatingGUIFromParameterNode = False

  def setup(self):
    # UI connection
    ScriptedLoadableModuleWidget.setup(self)
    uiWidget = slicer.util.loadUI(self.resourcePath('UI/Trajectory.ui'))  
    self.layout.addWidget(uiWidget)
    self.ui = slicer.util.childWidgetVariables(uiWidget)

    uiWidget.setMRMLScene(slicer.mrmlScene)

    self.logic = TrajectoryLogic()

    self.tagPlanNode = None
    self.tagPathNode = None
    self.targetFiducialsNode = None
    self.tagDestinationDispNode = None
    self.prevFiducialNode= None
    self.prevModelNode= None

    self.ui.VisiblityButton.checked= self.logic.SliceVisibility

    #Model Node 
    self.ModelSelector = slicer.qMRMLNodeComboBox()
    self.ModelSelector.nodeTypes=(("vtkMRMLModelNode"), "")
    self.ModelSelector.baseName=("Path")
    self.ModelSelector.selectNodeUponCreation = True
    self.ModelSelector.showChildNodeTypes = False
    self.ModelSelector.setMRMLScene( slicer.mrmlScene )
  
    #UI Connections
    self.ui.Planselector.connect("currentNodeChanged(vtkMRMLNode*)", self.onPlanselector)
    self.ui.VisiblityButton.connect('toggled(bool)',self.onVisiblityButton)
    self.ui.EntryPointButton.connect('clicked(bool)', self.onEntryPointButton)
    self.ui.TargetPointButton.connect('clicked(bool)', self.onTargetPointButton)
    self.ui.TargetPointJumpSlicesButton.connect('clicked(bool)', self.onTargetJumpSlicesButton)
    self.ui.EntryPointJumpSlicesButton.connect('clicked(bool)', self.onEntryJumpSlicesButton)
    self.ui.VolumeReslicerButton.connect('clicked(bool)', self.onVolumereSlicer)
    self.ui.RadiusSliderWidget.connect("valueChanged(double)", self.onTubeUpdated)
    self.ui.lengthLineEdit.connect("currentNodeChanged(vtkMRMLNode*)", self.onlengthLineEdit)

  def cleanup(self):
    pass

  def onPlanselector(self):
    #Check and hide the Fiducial markers and model
    if (self.prevFiducialNode != None and self.prevModelNode!= None):
      self.prevModelNode.SetDisplayVisibility(0)

    
    if (self.prevFiducialNode != None):
      self.logic.setVolumeReslicer(False)
      self.ui.VolumeReslicerButton.checked= False
      self.prevFiducialNode.SetDisplayVisibility(0)

    # Remove observer if previous node exists
    if self.logic.PlanNode and self.tagPlanNode:
      self.logic.PlanNode.RemoveObserver(self.tagPlanNode)

    # Update selected node, add observer, and update control points
    if self.ui.Planselector.currentNode():
      self.logic.PlanNode = self.ui.Planselector.currentNode()

      # Check if model has already been generated with for this fiducial list
      tubeModelID = self.logic.PlanNode.GetAttribute('Trajectory.Plan')
      self.ModelSelector.setCurrentNodeID(tubeModelID)
      self.tagPlanNode = self.logic.PlanNode.AddObserver(slicer.vtkMRMLMarkupsNode.PointModifiedEvent, self.logic.controlPointsUpdated, 2)
    
    # Set the visibility of current node
    if (self.ui.Planselector.currentNode() != None and self.ModelSelector.currentNode() != None):
      self.ui.Planselector.currentNode().SetDisplayVisibility(1)
      self.ModelSelector.currentNode().SetDisplayVisibility(1)

    # Update checkbox
    if (self.ui.Planselector.currentNode() == None or self.ModelSelector.currentNode() == None):
      self.ui.EntryPointButton.enabled = True
      self.ui.TargetPointButton.enabled = True
    else:
      self.logic.PlanNode.SetAttribute('Trajectory.Plan',self.logic.PathNode.GetID())
      self.logic.updateCurve()
    self.prevFiducialNode= self.ui.Planselector.currentNode()
    self.prevModelNode= self.ModelSelector.currentNode()


  def onVisiblityButton(self, state):
    # Visibility of the model in slices
    self.logic.SliceVisibility= state
    modelDisplayNode = self.ModelSelector.currentNode().GetDisplayNode()
    if self.logic.SliceVisibility:
      modelDisplayNode.SetVisibility2D(True)
    else:
      modelDisplayNode.SetVisibility2D(False)

  def onEntryPointButton(self):
    # Entry point placement
    plan1 = self.ui.Planselector.currentNode()
    slicer.modules.markups.logic().StartPlaceMode(0)
    self.entryID= (plan1.GetNumberOfFiducials())
    plan1.AddObserver(slicer.vtkMRMLMarkupsNode.PointAddedEvent, self.onEntryMarkupChanged)
    print(self.entryID)
    if self.entryID==1:
      self.onModelselector()
    self.logic.entryID= self.entryID

  def onEntryMarkupChanged(self, caller, event):
    # set label for entry point
    plan1= caller
    plan1.SetNthFiducialLabel(self.entryID,"Entry Point")
    self.ui.EntryPointButton.enabled = False

  def onTargetPointButton(self):
    # Target point placement
    plan1 = self.ui.Planselector.currentNode()
    slicer.modules.markups.logic().StartPlaceMode(0)
    self.targetID= (plan1.GetNumberOfFiducials())
    plan1.AddObserver(slicer.vtkMRMLMarkupsNode.PointAddedEvent, self.onTargetMarkupChanged)
    print(self.targetID)
    if self.targetID==1:
      self.onModelselector()
    self.logic.targetID= self.targetID

  def onTargetMarkupChanged(self, caller, event):
    # sets label for target point
    plan1= caller
    plan1.SetNthFiducialLabel(self.targetID,"Target Point")
    self.ui.TargetPointButton.enabled = False


  def onEntryJumpSlicesButton(self):
    # Jumps the entry points to slices
    self.logic.setJumpToEntryPoint()

  def onTargetJumpSlicesButton(self):
    # Jumps the target point to slices
    self.logic.setJumpToTargetPoint()
    

  def onVolumereSlicer(self, state):
    self.logic.setVolumeReslicer(state)

  def onModelselector(self):
    # Create a new node for model
    self.model=self.ModelSelector.addNode("vtkMRMLModelNode")
    if self.logic.PathNode and self.tagPathNode:
      self.logic.PathNode.RemoveObserver(self.tagPathNode)
      if self.logic.PathNode.GetDisplayNode() and self.tagDestinationDispNode:
        self.logic.PathNode.GetDisplayNode().RemoveObserver(self.tagDestinationDispNode)
    
    # Update destination node
    if self.ModelSelector.currentNode():
      self.logic.PathNode = self.ModelSelector.currentNode()
      self.tagPathNode = self.logic.PathNode.AddObserver(vtk.vtkCommand.ModifiedEvent, self.onModelModifiedEvent)

      if self.logic.PathNode.GetDisplayNode():
        self.tagDestinationDispNode = self.logic.PathNode.GetDisplayNode().AddObserver(vtk.vtkCommand.ModifiedEvent, self.onModelDisplayModifiedEvent)

    # Update checkbox
    if (self.ui.Planselector.currentNode() != None and self.ModelSelector.currentNode() != None):
      self.logic.PlanNode.SetAttribute('Trajectory.Plan',self.logic.PathNode.GetID())
      self.logic.updateCurve()

  def onTubeUpdated(self):
    #Path Radius
    self.logic.setTubeRadius(self.ui.RadiusSliderWidget.value)

  def onModelModifiedEvent(self, caller, event):
    self.ui.lengthLineEdit.text = '%.2f' % self.logic.CurveLength

  def onlengthLineEdit(self):
    self.ui.lengthLineEdit.text = '%.2f' % self.logic.CurveLength


#
# Logic
#

class TrajectoryLogic:

  def __init__(self):
    self.PlanNode = None
    self.PathNode = None
    self.TubeRadius = 2.0

    self.AutomaticUpdate = True
    self.SliceVisibility= True

    self.CurvePoly = None
    self.interpResolution = 25
    self.CurveLength = -1.0  ## Length of the curve (<0 means 'not measured')
    self.transformMatrix= vtk.vtkMatrix4x4()
    self.entryID=0
    self.targetID=1
    self.transformNode = slicer.vtkMRMLTransformNode()
    slicer.mrmlScene.AddNode(self.transformNode)

  def setJumpToEntryPoint(self):
    slicer.modules.markups.logic().JumpSlicesToNthPointInMarkup(self.PlanNode.GetID(), self.entryID)

  def setJumpToTargetPoint(self):
    slicer.modules.markups.logic().JumpSlicesToNthPointInMarkup(self.PlanNode.GetID(), self.targetID)

  def setTubeRadius(self, radius):
    self.TubeRadius = radius
    self.updateCurve()  

  def generateCurveOnce(self):
    prevAutomaticUpdate = self.AutomaticUpdate
    self.AutomaticUpdate = True
    self.updateCurve()
    self.AutomaticUpdate = prevAutomaticUpdate

  def controlPointsUpdated(self,caller,event):
    self.updateCurve()

  def setVolumeReslicer(self, state):
    if state:
      n0= numpy.zeros(3)
      n1= numpy.zeros(3)
      self.PlanNode.GetNthControlPointPosition(self.entryID, n0)
      self.PlanNode.GetNthControlPointPosition(self.targetID, n1)
      v= (n0-n1)/ numpy.linalg.norm(n1-n0)
      x= numpy.random.randn(3)
      x= x- x.dot(v) *v
      x/= numpy.linalg.norm(x)
      y= numpy.cross(v,x)
      y=numpy.append(y,numpy.zeros(1))
      y= numpy.append(y,numpy.append(x,numpy.zeros(1)) )
      y= numpy.append(y, numpy.append(v,numpy.zeros(1)))
      y= numpy.append(y, numpy.identity(4)[:,3], axis=0)
      y=numpy.reshape(y, (4,4))
      y= numpy.transpose(y)
      for i in range(0, 4):
            for j in range(0, 4):
                self.transformMatrix.SetElement(i, j, y[i, j])

    else:
      identity=numpy.identity(4)
      for i in range(0, 4):
            for j in range(0, 4):
                self.transformMatrix.SetElement(i, j, identity[i, j])
    self.setTransform()
    
  def setTransform(self):
    VolumeNode=slicer.mrmlScene.GetNthNodeByClass(0,"vtkMRMLScalarVolumeNode")
    VolumeNode.SetAndObserveTransformNodeID(self.transformNode.GetID())
    self.transformNode.SetMatrixTransformToParent(self.transformMatrix)
    layoutManager = slicer.app.layoutManager()
    for sliceViewName in layoutManager.sliceViewNames():
      layoutManager.sliceWidget(sliceViewName).mrmlSliceNode().RotateToVolumePlane(VolumeNode)
      layoutManager.sliceWidget(sliceViewName).sliceController().setSliceVisible(True)
    self.setJumpToTargetPoint()
    

  def nodeToPolyCardinalSpline(self, PlanNode, outputPoly):

    nOfControlPoints = PlanNode.GetNumberOfFiducials()
    pos = [0.0, 0.0, 0.0]

    # One spline for each direction.
    aSplineX = vtk.vtkCardinalSpline()
    aSplineY = vtk.vtkCardinalSpline()
    aSplineZ = vtk.vtkCardinalSpline()

    aSplineX.ClosedOff()
    aSplineY.ClosedOff()
    aSplineZ.ClosedOff()

    for i in range(0, nOfControlPoints):
      PlanNode.GetNthFiducialPosition(i, pos)
      aSplineX.AddPoint(i, pos[0])
      aSplineY.AddPoint(i, pos[1])
      aSplineZ.AddPoint(i, pos[2])
    
    # Interpolate x, y and z by using the three spline filters and
    # create new points
    nInterpolatedPoints = (self.interpResolution+2)*(nOfControlPoints-1) # One section is divided into self.interpResolution segments
    points = vtk.vtkPoints()
    r = [0.0, 0.0]
    aSplineX.GetParametricRange(r)
    t = r[0]
    p = 0
    tStep = (nOfControlPoints-1.0)/(nInterpolatedPoints-1.0)
    nOutputPoints = 0

    while t < r[1]:
      points.InsertPoint(p, aSplineX.Evaluate(t), aSplineY.Evaluate(t), aSplineZ.Evaluate(t))
      t = t + tStep
      p = p + 1
    nOutputPoints = p
    
    lines = vtk.vtkCellArray()
    lines.InsertNextCell(nOutputPoints)
    for i in range(0, nOutputPoints):
      lines.InsertCellPoint(i)
        
    outputPoly.SetPoints(points)
    outputPoly.SetLines(lines)

  def pathToPoly(self, path, poly):
    points = vtk.vtkPoints()
    cellArray = vtk.vtkCellArray()

    points = vtk.vtkPoints()
    poly.SetPoints(points)

    lines = vtk.vtkCellArray()
    poly.SetLines(lines)

    linesIDArray = lines.GetData()
    linesIDArray.Reset()
    linesIDArray.InsertNextTuple1(0)
    
    polygons = vtk.vtkCellArray()
    poly.SetPolys( polygons )
    idArray = polygons.GetData()
    idArray.Reset()
    idArray.InsertNextTuple1(0)
    
    for point in path:
      pointIndex = points.InsertNextPoint(*point)
      linesIDArray.InsertNextTuple1(pointIndex)
      linesIDArray.SetTuple1( 0, linesIDArray.GetNumberOfTuples() - 1 )
      lines.SetNumberOfCells(1)

  def calculateLineLength(self, poly):
    lines = poly.GetLines()
    points = poly.GetPoints()
    pts = vtk.vtkIdList()

    lines.GetCell(0, pts)
    ip = numpy.array(points.GetPoint(pts.GetId(0)))
    n = pts.GetNumberOfIds()

    # Check if there is overlap between the first and last segments
    # (for making sure to close the loop for spline curves)
    if n > 2:
      slp = numpy.array(points.GetPoint(pts.GetId(n-2)))
      # Check distance between the first point and the second last point
      if numpy.linalg.norm(slp-ip) < 0.00001:
        n = n - 1
        
    length = 0.0
    pp = ip
    for i in range(1,n):
      p = numpy.array(points.GetPoint(pts.GetId(i)))
      length = length + numpy.linalg.norm(pp-p)
      pp = p

    return length

  def updateCurve(self):

    # if self.AutomaticUpdate == False:
    #   return

    if self.PlanNode and self.PathNode:

      if self.PlanNode.GetNumberOfFiducials() < 2:
        if self.CurvePoly != None:
          self.CurvePoly.Initialize()

        self.CurveLength = 0.0

      else:
        if self.CurvePoly == None:
          self.CurvePoly = vtk.vtkPolyData()
        
        if self.PathNode.GetDisplayNodeID() == None:
          modelDisplayNode = slicer.vtkMRMLModelDisplayNode()
          self.ModelColor = numpy.random.randint(0,9,3)  #Generate a list of 3 random element for the color of model
          modelDisplayNode.SetColor(self.ModelColor)
          slicer.mrmlScene.AddNode(modelDisplayNode)
          modelDisplayNode.SetVisibility2D(self.SliceVisibility) #Set the 2D visibility of the model in the slices
          self.PathNode.SetAndObserveDisplayNodeID(modelDisplayNode.GetID())

        # Cardinal Spline
        self.nodeToPolyCardinalSpline(self.PlanNode, self.CurvePoly)
        self.CurveLength = self.calculateLineLength(self.CurvePoly)

      tubeFilter = vtk.vtkTubeFilter()
      curvatureValues = vtk.vtkDoubleArray()

      tubeFilter.SetInputData(self.CurvePoly)
      tubeFilter.SetRadius(self.TubeRadius)
      tubeFilter.SetNumberOfSides(20)
      tubeFilter.CappingOn()
      tubeFilter.Update()

      self.PathNode.SetAndObservePolyData(tubeFilter.GetOutput())
      self.PathNode.Modified()
      
      if self.PathNode.GetScene() == None:
        slicer.mrmlScene.AddNode(self.PathNode)

      displayNode = self.PathNode.GetDisplayNode()
      if displayNode:
        displayNode.SetActiveScalarName('')

  def distanceToPoint(self, point, extrapolate):

    # distanceToPoint() calculates the approximate minimum distance between
    # the specified point and the closest segment of the curve.
    # It calculates the minimum distance between the point and each segment
    # of the curve (approxmated as a straight line) and select the segment with
    # the minimum distance from the point as a closest segment.

    npoint = numpy.array(point)

    if self.CurvePoly == None:
      return numpy.Inf

    lines = self.CurvePoly.GetLines()
    points = self.CurvePoly.GetPoints()
    pts = vtk.vtkIdList()

    lines.GetCell(0, pts)
    ip = numpy.array(points.GetPoint(pts.GetId(0)))
    n = pts.GetNumberOfIds()

    # First point on the segment
    p1 = ip

    minMag2 = numpy.Inf
    minIndex = -1
    minErrVec = numpy.array([0.0, 0.0, 0.0])

    errVec = numpy.array([0.0, 0.0, 0.0])
    for i in range(1,n):
      # Second point on the segment
      p2 = numpy.array(points.GetPoint(pts.GetId(i)))

      # Normal vector along the segment
      nvec = p2-p1
      norm = numpy.linalg.norm(nvec)
      if norm != 0:
        nnvec = nvec / norm

      # Calculate the distance between the point and the segment
      mag2 = 0.0

      op = npoint - p1
      aproj = numpy.inner(op, nnvec)

      if extrapolate and ((i == 1 and aproj < 0.0) or (i == n-1 and aproj > 0.0)):
        # extrapolate first or last segment
        errVec = op-aproj*nnvec  # perpendicular
        mag2 = numpy.inner(errVec,errVec) # magnitude^2
      else:
        if aproj < 0.0:
          errVec = npoint - p1
          mag2 = numpy.inner(errVec, errVec) # magnitude^2
        elif aproj > norm:
          errVec = npoint - p2
          mag2 = numpy.inner(errVec, errVec) # magnitude^2
        else:
          errVec = op-aproj*nnvec # perpendicular
          mag2 = numpy.inner(errVec,errVec) # magnitude^2
        
      if mag2 < minMag2:
        minMag2 = mag2
        minIndex = i
        minErrVec = errVec
    
      p1 = p2

    distance = numpy.sqrt(minMag2)

    return (distance, minErrVec)
