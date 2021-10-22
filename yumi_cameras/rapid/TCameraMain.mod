MODULE TCameraMain
!======================================================================================================
! Software License Agreement (BSD License) 
!
! Copyright (c) 2015, ABB
! All rights reserved.
!
! Redistribution and use in source and binary forms, with
! or without modification, are permitted provided that 
! the following conditions are met:
!
!    * Redistributions of source code must retain the 
!      above copyright notice, this list of conditions 
!      and the following disclaimer.
!    * Redistributions in binary form must reproduce the 
!      above copyright notice, this list of conditions 
!      and the following disclaimer in the documentation 
!      and/or other materials provided with the 
!      distribution.
!    * Neither the name of ABB nor the names of its 
!      contributors may be used to endorse or promote 
!      products derived from this software without 
!      specific prior written permission.
!
! THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
! AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
! IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
! ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
! LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
! DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
! SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
! CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
! OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF 
! THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
!======================================================================================================




    !==================================================================================================
    ! Data components
    !==================================================================================================
    !-------------------------------------------------
    ! Records
    !-------------------------------------------------
    LOCAL RECORD CAMERA_RECORD
        num exposure_time;
    ENDRECORD
    
    LOCAL RECORD CAMERA_DEVICE_RECORD
        bool      present;
        cameradev device;
    ENDRECORD
    
    LOCAL RECORD CAMERA_DEVICES_RECORD
        CAMERA_DEVICE_RECORD left;
        CAMERA_DEVICE_RECORD right;
    ENDRECORD
    
    !-------------------------------------------------
    ! Variables and constants
    !-------------------------------------------------
    LOCAL CONST num DEFAULT_EXPOSURE_TIME := 8;
    
    ! Default prefix and suffixes for camera devices.
    LOCAL CONST string DEFAULT_PREFIX := "HandCam";
    LOCAL CONST string DEFAULT_SUFFIX_R := "_R";
    LOCAL CONST string DEFAULT_SUFFIX_L := "_L";
    
    ! Containers for data used in the camera instructions.
    LOCAL VAR CAMERA_DEVICES_RECORD camera_devices;
    LOCAL VAR CAMERA_RECORD camera_data_left;
    LOCAL VAR CAMERA_RECORD camera_data_right;
    
    ! Components for specifing which camera to use.
    LOCAL CONST num NO_CAMERA    := 0;
    LOCAL CONST num LEFT_CAMERA  := 1;
    LOCAL CONST num RIGHT_CAMERA := 2;
    LOCAL CONST num BOTH_CAMERAS := 3;
    LOCAL VAR num use_camera;
    
    ! Components for externally triggering camera instructions.
    LOCAL CONST num NONE          := 0;
    LOCAL CONST num REQUEST_IMAGE := 1;
    LOCAL CONST num SET_EXPOSURE  := 2;
    LOCAL VAR num camera_current_command;
    LOCAL VAR intnum camera_interupt;
    
    LOCAL CONST string left_cam_job := "L_CaptureAndFTP.job";
    LOCAL CONST string right_cam_job := "R_CaptureAndFTP.job";
    
    
    !==================================================================================================
    ! Primary components
    !==================================================================================================
    PROC main()
        ! Initialize the camera module.
        initializeCameraModule;
        
        TPWrite "Cameras initialized";
        
        WHILE TRUE DO
            WaitTime 1;
        ENDWHILE
    ENDPROC
    
    LOCAL PROC requestImages()
        TPWrite "Requesting images...";
        TEST use_camera
            CASE LEFT_CAMERA:
                requestImage camera_devices.left;
                ! TPWrite "Left cam image request";
                
            CASE RIGHT_CAMERA:
                requestImage camera_devices.right;
                ! TPWrite "Right cam image request";
            
            CASE BOTH_CAMERAS:
                !TPWrite "Both cams image request";
                !TPWrite "Left";
                requestImage camera_devices.left;
                !TPWrite "Right";
                requestImage camera_devices.right;
                
        
            DEFAULT:
                ! Do nothing.
        ENDTEST
    ENDPROC
    
    LOCAL PROC requestImage(VAR CAMERA_DEVICE_RECORD device)
        IF device.present THEN
            CamReqImage device.device;
            !TPWrite "Image requested";
        ENDIF
    ENDPROC
    
    LOCAL PROC setExposures()
        TEST use_camera
            CASE LEFT_CAMERA:
                setExposure camera_devices.left, camera_data_left;
                
            CASE RIGHT_CAMERA:
                setExposure camera_devices.right, camera_data_right;
            
            CASE BOTH_CAMERAS:
                setExposure camera_devices.left, camera_data_left;
                setExposure camera_devices.right, camera_data_right;
        
            DEFAULT:
                ! Do nothing.
        ENDTEST
    ENDPROC
    
    LOCAL PROC setExposure(VAR CAMERA_DEVICE_RECORD device, VAR CAMERA_RECORD data)
        IF device.present THEN
            CamSetExposure device.device \ExposureTime:=data.exposure_time;
        ENDIF
    ENDPROC
    
    !-------------------------------------------------
    ! Interupt handlers
    !-------------------------------------------------
    LOCAL TRAP runCameraCommandHandler
        !TPWrite "TRAP: Camera command handler";
        checkData camera_data_left;
        checkData camera_data_right;
        runCommand;
        camera_current_command := NONE;
    ENDTRAP
    
    
    
    
    !==================================================================================================
    ! Auxiliary components
    !==================================================================================================    
    !-------------------------------------------------
    ! Auxiliary procedures
    !------------------------------------------------- 
    LOCAL PROC initializeCameraModule()
        ! Set some default values for the camera instructions.
        setDefaultData camera_data_left;
        setDefaultData camera_data_right;
        
        ! Find out if there are camera devices present.
        !
        ! Note: This example assumes there to only be up to two camera devices present.
        !       I.e. possibly one for the left SmartGripper and/or possibly one for the right SmartGripper.
        findCameraDevices;
        
        use_camera := BOTH_CAMERAS;
        camera_current_command := NONE;

        setCameraRunMode;
        
        ! Setup a interrupt signal
        IDelete camera_interupt;
        CONNECT camera_interupt WITH runCameraCommandHandler;
        ISignalDI RUN_CAMERA_COMMAND, HIGH, camera_interupt;
    ENDPROC
    
    LOCAL PROC setDefaultData(VAR CAMERA_RECORD data)
        data.exposure_time := DEFAULT_EXPOSURE_TIME;
    ENDPROC
    
    LOCAL PROC findCameraDevices()
        VAR datapos block;
        VAR string name;
        TPWrite "Searching for gripper cameras...";
        
        camera_devices.left.present  := FALSE;
        camera_devices.right.present := FALSE;
        
        ! Setup a search for all cameradev variables starting with DEFAULT_PREFIX.
        SetDataSearch "cameradev"
                      \Object:=DEFAULT_PREFIX + ".*";
        
        TPWrite "Expecting " + DEFAULT_PREFIX + DEFAULT_SUFFIX_L;
        TPWrite "Expecting " + DEFAULT_PREFIX + DEFAULT_SUFFIX_R;

                      
      ! Look for up to two camera devices.
        FOR i FROM 0 TO 1 DO
            IF GetNextSym(name, block) THEN
                TPWrite "Device name:" + name;
                IF name =  DEFAULT_PREFIX + DEFAULT_SUFFIX_L THEN
                    camera_devices.left.present := TRUE;
                    GetDataVal name
                               \Block:=block,
                               camera_devices.left.device;
                    CamSetProgramMode camera_devices.left.device;
                    CamLoadJob camera_devices.left.device, left_cam_job;
                    TPWrite "Left camera found!";
                               
                ELSEIF name =  DEFAULT_PREFIX + DEFAULT_SUFFIX_R THEN
                    camera_devices.right.present := TRUE;
                    GetDataVal name
                               \Block:=block,
                               camera_devices.right.device;
                    CamSetProgramMode camera_devices.right.device;
                    CamLoadJob camera_devices.right.device, right_cam_job;
                    TPWrite "Right camera found!";
                          
                ENDIF
            ENDIF
        ENDFOR
    
    ENDPROC
 
    LOCAL PROC setCameraRunMode()
        TPWrite "Setting cameras run mode...";
        IF camera_devices.left.present THEN
            CamSetRunMode camera_devices.left.device;
        ENDIF
        
        IF camera_devices.right.present THEN
            CamSetRunMode camera_devices.right.device;
        ENDIF
    ENDPROC
    
    
    LOCAL PROC runCommand()
        IF use_camera <> NO_CAMERA THEN
            TPWrite "Running camera command";
            TEST camera_current_command
                CASE REQUEST_IMAGE:
                    IF RobOS() THEN
                        requestImages;
                    ELSE
                        TPWrite "Camera: Request Image";
                    ENDIF
                    
                CASE SET_EXPOSURE:
                    IF RobOS() THEN
                        setExposures;
                    ELSE
                        TPWrite "Camera: Set Exposure Time";
                    ENDIF
                    
                DEFAULT:
                    ! Do nothing.
                    TPWrite "Camera: Unknown command";
            ENDTEST
        ENDIF
    ENDPROC
    
    LOCAL PROC checkData(VAR CAMERA_RECORD data)
        IF data.exposure_time < 0.1 THEN
            data.exposure_time := 0.1;
        ENDIF
     ENDPROC
ENDMODULE