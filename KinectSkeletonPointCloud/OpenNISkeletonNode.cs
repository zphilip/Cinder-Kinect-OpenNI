#region usings

using System;
using System.ComponentModel.Composition;
using System.Runtime.InteropServices;
using System.Collections.Generic;
using System.Threading;
using System.Drawing;
using System.Drawing.Imaging;


using SlimDX;
using SlimDX.Direct3D9;
using SlimDX.Direct2D;

using VVVV.Core.Logging;
using VVVV.PluginInterfaces.V1;
using VVVV.PluginInterfaces.V2;
using VVVV.PluginInterfaces.V2.EX9;
using VVVV.Utils.VColor;
using VVVV.Utils.VMath;
using VVVV.Utils.SlimDX;

using xn;

#endregion usings

//here you can change the vertex type
using VertexType = VVVV.Utils.SlimDX.TexturedVertex;



namespace VVVV.Nodes
{
    #region PluginInfo
    [PluginInfo(Name = "Skeleton", Category = "OpenNI", Help = "OpenNi MultiSkeleton Reader", Tags = "plugin, OpenNI,Kinect")]
    #endregion PluginInfo

    public class OpenNISkeletonNode : DXTextureOutPluginBase, IPluginEvaluate
    {

        #region fields & pins

        [Input("Config", DefaultString = "")]
        IDiffSpread<string> FPinInConfig; 

        [Input("Texturing")]
        IDiffSpread<bool> FIsRenderingIn;

        [Input("RenderBackground")]
        IDiffSpread<bool> FRenderBackgroundIn;

        [Input("RenderLabels")]
        IDiffSpread<bool> FRenderLabelsIn;

        [Input("PrintId")]
        IDiffSpread<bool> FPrintIdIn;

        [Input("CustomColors")]
        IDiffSpread<bool> FCustomColorsIn;

        [Input("ConfidenceTS", DefaultValue = 0.5)]
        ISpread<double> FConfidenceTSIn;

        [Input("BaseColor")]
        IDiffSpread<RGBAColor> FBaseColorIn;

        [Input("TrackingColor")]
        IDiffSpread<RGBAColor> FTrackingColorIn;

        [Output("Message")]
        ISpread<string> FMessage;

        [Output("TrackedUsersId")]
        ISpread<int> FTrackedUsersId;

        [Output("JointsPosition")]
        ISpread<ISpread<Vector3D>> FJointsPosition;

        [Import()]
        ILogger FLogger;

        //track the current texture slice
        int FCurrentSlice;

        #endregion fields & pins
        #region Kinect vars

       // public readonly string SAMPLE_XML_FILE = @"D:/developpa/KINECT/VVVV/Plugin/OpenNISkeleton/Data/SamplesConfig.xml";
        public readonly string SAMPLE_XML_FILE = @"plugins/OpenNISkeleton/Data/SamplesConfig.xml";
        
        //public string SAMPLE_XML_FILE ;

        private Context context;
        private xn.DepthGenerator depth;
        private UserGenerator userGenerator;
        private SkeletonCapability skeletonCapbility;
        private PoseDetectionCapability poseDetectionCapability;
        private string calibPose;
        private Thread readerThread;
        private bool shouldRun;
        private System.Drawing.Bitmap bitmap;
        private int[] histogram;
        private bool reading;

        private Dictionary<uint, Dictionary<SkeletonJoint, SkeletonJointPosition>> joints;

        private bool shouldDrawPixels = true;
        private bool shouldDrawBackground = true;
        private bool shouldPrintID = true;
        private bool shouldPrintState = true;
        private bool drawLabels = true;

        private bool kinectStarted = false;

        private List<uint> trackedUsers;

        private int texW;
        private int texH;

        private bool setCustomColors = false;

        private Color baseColor = Color.Turquoise;
        private Color trackingColor = Color.OrangeRed;
        private Color calibratingColor = Color.PaleGreen;

        private Color[] colors = { Color.Blue, Color.Orange, Color.ForestGreen, Color.Yellow, Color.Orange, Color.Purple, Color.White };
        private Color[] anticolors = { Color.Red, Color.White, Color.Red, Color.Purple, Color.Blue, Color.Yellow, Color.Black };
        private int ncolors = 6;

        private IntPtr bitmapPointer;

        #endregion

        // import host and hand it to base constructor
        [ImportingConstructor()]
        public OpenNISkeletonNode(IPluginHost host)
            : base(host)
        {
        }

        #region KINECT FUNCTIONZ
        private void getSkeleton(uint user, int userIndex)
        {
          
            GetJoints(user);
            Dictionary<SkeletonJoint, SkeletonJointPosition> dict = this.joints[user];

            FJointsPosition[userIndex].SliceCount = dict.Count;

            int jointIndex = 0;

            foreach (KeyValuePair<SkeletonJoint, SkeletonJointPosition> kvp in dict)
            {
                SkeletonJointPosition obj = kvp.Value;

                if (obj.fConfidence >= FConfidenceTSIn[0])
                {
                    FJointsPosition[userIndex][jointIndex] = new Vector3D(obj.position.X, obj.position.Y, obj.position.Z);
                }
                jointIndex++;
            }
        }
        public void initKinect()
        {

            kinectStarted = true;

            FMessage[0] = "Init OpenNI Node";

            this.context = new Context(SAMPLE_XML_FILE);
            this.depth = context.FindExistingNode(xn.NodeType.Depth) as DepthGenerator;

            if (this.depth == null)
            {
                //throw new Exception("Viewer must have a depth node!");
                FLogger.Log(LogType.Debug, "non ci siamo eh");
            }
            else
            {
                FLogger.Log(LogType.Debug, "Ecchime sono un Primesense attivato e running....:P");
            }

            this.userGenerator = new UserGenerator(this.context);
            this.skeletonCapbility = new SkeletonCapability(this.userGenerator);
            this.poseDetectionCapability = new PoseDetectionCapability(this.userGenerator);
            this.calibPose = this.skeletonCapbility.GetCalibrationPose();

            this.userGenerator.NewUser += new UserGenerator.NewUserHandler(userGenerator_NewUser);
            this.userGenerator.LostUser += new UserGenerator.LostUserHandler(userGenerator_LostUser);
            this.poseDetectionCapability.PoseDetected += new PoseDetectionCapability.PoseDetectedHandler(poseDetectionCapability_PoseDetected);
            this.skeletonCapbility.CalibrationEnd += new SkeletonCapability.CalibrationEndHandler(skeletonCapbility_CalibrationEnd);

            this.skeletonCapbility.SetSkeletonProfile(SkeletonProfile.All);
            this.joints = new Dictionary<uint, Dictionary<SkeletonJoint, SkeletonJointPosition>>();
            this.userGenerator.StartGenerating();

            FMessage[0] = "OpenNI Node registered";

            this.histogram = new int[this.depth.GetDeviceMaxDepth()];

            MapOutputMode mapMode = this.depth.GetMapOutputMode();

            texW = (int)mapMode.nXRes;
            texH = (int)mapMode.nYRes;
            FLogger.Log(LogType.Debug, "Winit: " + texW + "Hinit: " + texH);
            FLogger.Log(LogType.Debug, "kinectStarted = " + kinectStarted);
            this.bitmap = new System.Drawing.Bitmap((int)mapMode.nXRes, (int)mapMode.nYRes, System.Drawing.Imaging.PixelFormat.Format32bppArgb);


            this.shouldRun = true;
            this.readerThread = new Thread(ReaderThread);
            this.readerThread.Start();

            FMessage[0] = "OpenNI Thread Started ";

        }

        private unsafe void CalcHist(DepthMetaData depthMD)
        {
            // reset

            for (int i = 0; i < this.histogram.Length; ++i)
                this.histogram[i] = 0;

            ushort* pDepth = (ushort*)depthMD.DepthMapPtr.ToPointer();

            int points = 0;
            for (int y = 0; y < depthMD.YRes; ++y)
            {
                for (int x = 0; x < depthMD.XRes; ++x, ++pDepth)
                {
                    ushort depthVal = *pDepth;
                    if (depthVal != 0)
                    {
                        this.histogram[depthVal]++;
                        points++;
                    }
                }
            }

            for (int i = 1; i < this.histogram.Length; i++)
            {
                this.histogram[i] += this.histogram[i - 1];
            }

            if (points > 0)
            {
                for (int i = 1; i < this.histogram.Length; i++)
                {
                    this.histogram[i] = (int)(256 * (1.0f - (this.histogram[i] / (float)points)));
                }
            }
        }
        private unsafe Color setUserColor(ushort label)
        {
            Color color = trackingColor;
            if (trackedUsers.Contains(label))
            {
                color = trackingColor;
            }
            else
            {
                color = baseColor;
            }
            /*
            if (this.skeletonCapbility.IsCalibrating(label))
            {
                color = calibratingColor;

            }else{
                if (this.skeletonCapbility.IsTracking(label))
                {
                    color = trackingColor;
                }
                else {
                    color = baseColor;
                }
            }
             * */
           
            return color;
        }
        private unsafe void ReaderThread()
        {
           
            while (this.shouldRun)
            {

                lock (this)
                {

                    try
                    {
                        this.context.WaitOneUpdateAll(this.depth);
                    }
                    catch (Exception)
                    {

                    }

                    if (this.shouldDrawPixels)
                    {
                        DepthMetaData depthMD = new DepthMetaData();

                        this.depth.GetMetaData(depthMD);

                        CalcHist(depthMD);
                        
                        Rectangle rect = new Rectangle(0, 0, this.bitmap.Width, this.bitmap.Height);
                        BitmapData data = this.bitmap.LockBits(rect, ImageLockMode.WriteOnly, System.Drawing.Imaging.PixelFormat.Format32bppArgb);

                  
                        ushort* pDepth = (ushort*)this.depth.GetDepthMapPtr().ToPointer();
                        ushort* pLabels = (ushort*)this.userGenerator.GetUserPixels(0).SceneMapPtr.ToPointer();
                        
                        // set pixels
                        for (int y = 0; y < texH; ++y)
                        {
                            byte* pDest = (byte*)data.Scan0.ToPointer() + y * data.Stride;
                            for (int x = 0; x < texW; ++x, ++pDepth, ++pLabels, pDest += 4)
                            {

                                pDest[0] = pDest[1] = pDest[2]=0;
                                pDest[3] = 255;
                                
                                ushort label = *pLabels;

                                if (this.shouldDrawBackground || *pLabels != 0)
                                {

                                    Color labelColor = Color.White;

                                    if (label != 0 && drawLabels)
                                    {
                                        if (setCustomColors)
                                        {
                                            labelColor = setUserColor(label);
                                        }
                                        else {
                                            labelColor = colors[label % ncolors];
                                        }
                                       //
                                    }

                                    byte pixel = (byte)this.histogram[*pDepth];
                                    
                                    pDest[0] = (byte)(pixel * (labelColor.B / 256.0));
                                    pDest[1] = (byte)(pixel * (labelColor.G / 256.0));
                                    pDest[2] = (byte)(pixel * (labelColor.R / 256.0));
                                    
                                }
                            }

                        }
                        
                        this.bitmap.UnlockBits(data);

                        reading = true;
                        bitmapPointer = data.Scan0;
                        reading = false;


                        uint[] users = this.userGenerator.GetUsers();
                        //trackedUsers = new List<uint>();

                        Graphics g = Graphics.FromImage(this.bitmap);

                        foreach (uint user in users)
                        {
                            #region labels
                      
                            if (this.shouldPrintID)
                            {
                                Point3D com = this.userGenerator.GetCoM(user);
                                com = this.depth.ConvertRealWorldToProjective(com);

                                string label = "";
                                if (!this.shouldPrintState)
                                    label += user;
                                else if (this.skeletonCapbility.IsTracking(user))
                                    label += user + " - Tracking";
                                else if (this.skeletonCapbility.IsCalibrating(user))
                                    label += user + " - Calibrating...";
                                else
                                    label += user + " - Looking for pose";

                                g.DrawString(label, new System.Drawing.Font("Arial", 10), new SolidBrush(anticolors[user % ncolors]), com.X, com.Y);
                        
                            }
                            #endregion

                        }
                        g.Dispose();

                    }

                    uint[] userz = this.userGenerator.GetUsers();
                    trackedUsers = new List<uint>();

                    #region isolatemeplease
                    foreach (uint user in userz)
                     {
                         if (this.skeletonCapbility.IsTracking(user))
                         {
                             trackedUsers.Add(user);

                         }
                     }

                    int trackedUsersCount = trackedUsers.Count;
                    /// tracked user ID
                    FTrackedUsersId.SliceCount = trackedUsersCount;

                    for (int i = 0; i < trackedUsersCount; i++)
                    {
                        FTrackedUsersId[i] = (int)trackedUsers[i];
                    }

                    //////////////  Position spreading count
                    FJointsPosition.SliceCount = trackedUsersCount;

                    int index = 0;

                    foreach (uint trackedUser in trackedUsers)
                    {
                        getSkeleton(trackedUser, index);
                        index++;
                    }
                    #endregion
                   
                }
            }

        }


        private void GetJoint(uint user, SkeletonJoint joint)
        {
            SkeletonJointPosition pos = new SkeletonJointPosition();
            this.skeletonCapbility.GetSkeletonJointPosition(user, joint, ref pos);

            if (pos.position.Z == 0)
            {
                pos.fConfidence = 0;
            }
            else
            {
                pos.position = this.depth.ConvertRealWorldToProjective(pos.position);
            }
            this.joints[user][joint] = pos;

        }

        private void GetJoints(uint user)
        {
            GetJoint(user, SkeletonJoint.Head);
            GetJoint(user, SkeletonJoint.Neck);

            GetJoint(user, SkeletonJoint.LeftShoulder);
            GetJoint(user, SkeletonJoint.LeftElbow);
            GetJoint(user, SkeletonJoint.LeftHand);

            GetJoint(user, SkeletonJoint.RightShoulder);
            GetJoint(user, SkeletonJoint.RightElbow);
            GetJoint(user, SkeletonJoint.RightHand);

            GetJoint(user, SkeletonJoint.Torso);

            GetJoint(user, SkeletonJoint.LeftHip);
            GetJoint(user, SkeletonJoint.LeftKnee);
            GetJoint(user, SkeletonJoint.LeftFoot);

            GetJoint(user, SkeletonJoint.RightHip);
            GetJoint(user, SkeletonJoint.RightKnee);
            GetJoint(user, SkeletonJoint.RightFoot);
        }

        void skeletonCapbility_CalibrationEnd(ProductionNode node, uint id, bool success)
        {
            if (success)
            {
                this.skeletonCapbility.StartTracking(id);
                this.joints.Add(id, new Dictionary<SkeletonJoint, SkeletonJointPosition>());

                FMessage[0] = "Calibration ended for user " + id.ToString();
            }
            else
            {
                this.poseDetectionCapability.StartPoseDetection(calibPose, id);
                FMessage[0] = "Pose detection for user  " + id.ToString();
            }
        }

        void poseDetectionCapability_PoseDetected(ProductionNode node, string pose, uint id)
        {
            this.poseDetectionCapability.StopPoseDetection(id);
            this.skeletonCapbility.RequestCalibration(id, true);

            FMessage[0] = "Pose detected for user: " + id.ToString();
        }

        void userGenerator_NewUser(ProductionNode node, uint id)
        {
            this.poseDetectionCapability.StartPoseDetection(this.calibPose, id);

            FMessage[0] = "New user: " + id.ToString();
        }

        void userGenerator_LostUser(ProductionNode node, uint id)
        {
            this.joints.Remove(id);

            FMessage[0] = "Lost user: " + id.ToString();
        }


        //called when data for any output pin is requested
        public unsafe void Evaluate(int SpreadMax)
        {

            SetSliceCount(SpreadMax);

            if (!kinectStarted)
            {
                initKinect();
            }
            /*
            if (FPinInConfig.IsChanged)
            {
                if (!kinectStarted) { 
                    this.SAMPLE_XML_FILE = FPinInConfig[0];
                    if (SAMPLE_XML_FILE != null)
                       if (SAMPLE_XML_FILE.Length > 0)
                        initKinect();
                }
            }
            */

            if (FIsRenderingIn.IsChanged)
            {
                this.shouldDrawPixels = FIsRenderingIn[0];
            }
            if (FRenderBackgroundIn.IsChanged)
            {
                this.shouldDrawBackground = FRenderBackgroundIn[0];
            }
            if (FRenderLabelsIn.IsChanged)
            {
                this.drawLabels = FRenderLabelsIn[0];
            }
            if (FPrintIdIn.IsChanged)
            {
                this.shouldPrintID = FPrintIdIn[0];
            }
            if (FCustomColorsIn.IsChanged)
            {
                this.setCustomColors = FCustomColorsIn[0];
            }
            if (FBaseColorIn.IsChanged)
            {
                this.baseColor = FBaseColorIn[0].Color;
            }
            if (FTrackingColorIn.IsChanged)
            {
                this.trackingColor = FTrackingColorIn[0].Color;
            }

            Update();

        }

        //this method gets called, when Reinitialize() was called in evaluate,
        //or a graphics device asks for its data
        protected override Texture CreateTexture(int Slice, SlimDX.Direct3D9.Device device)
        {
            FLogger.Log(LogType.Debug, "Creating new texture at slice: " + Slice+" -> W: " + texW + "H: " + texH);
            return TextureUtils.CreateTexture(device, texW, texH);
        }

        //this method gets called, when Update() was called in evaluate,
        //or a graphics device asks for its texture, here you fill the texture with the actual data
        //this is called for each renderer, careful here with multiscreen setups, in that case
        //calculate the pixels in evaluate and just copy the data to the device texture here
        unsafe protected override void UpdateTexture(int Slice, Texture texture)
        {

            FCurrentSlice = Slice;

            var rect = texture.LockRectangle(0, LockFlags.Discard).Data;

            if (!reading)
            {
                rect.WriteRange(bitmapPointer, 640 * 480 * 4);
            }
            texture.UnlockRectangle(0);
        }
    }
}
        #endregion