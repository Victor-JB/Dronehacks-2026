"""Synchronized frames rendered during the actual physics run, never replayed poses."""
from pathlib import Path
import math
import numpy as np
import cv2,mujoco,imageio_ffmpeg
from PIL import Image,ImageDraw
from export_media import font,BG,TEXT,MUTED,CYAN,AMBER


class LiveVideo:
    def __init__(self,model,out):
        self.out=out;self.count=0
        self.renderer=mujoco.Renderer(model,800,1240)
        self.camera=mujoco.MjvCamera();self.camera.type=mujoco.mjtCamera.mjCAMERA_FREE
        self.camera.distance=3.5;self.camera.elevation=-68.;self.camera.azimuth=135.
        self.look=None
        self.option=mujoco.MjvOption();self.option.geomgroup[3]=0;self.option.geomgroup[4]=0
        self.writer=imageio_ffmpeg.write_frames(str(out/'uncut.mp4'),(1920,1080),fps=10,
            codec='libx264',quality=7,macro_block_size=1,ffmpeg_log_level='error',
            output_params=['-preset','fast','-movflags','+faststart'])
        self.writer.send(None)
        (out/'screenshots').mkdir(exist_ok=True)

    def frame(self,data,node,rgb,depth,position,yaw,t,distance,minimum):
        self.look=position.copy() if self.look is None else .95*self.look+.05*position
        self.camera.lookat[:]=self.look
        self.renderer.update_scene(data,camera=self.camera,scene_option=self.option)
        third=self.renderer.render().copy()
        canvas=Image.new('RGB',(1920,1080),BG);d=ImageDraw.Draw(canvas)
        d.text((28,18),'STANFORD DRONE HACKS / LIVE AUTONOMY',font=font(32,True),fill=TEXT)
        d.text((28,66),'MuJoCo physics • emulated ROS boundary • velocity servo, no PX4 • no hardware qualification',font=font(22),fill=AMBER)
        canvas.paste(Image.fromarray(third),(24,136))
        d.text((40,145),'3D simulator view / net cutaway for visibility',font=font(21),fill=TEXT)
        canvas.paste(Image.fromarray(rgb).resize((600,375)),(1296,139))
        d.text((1302,110),'FORWARD RGB / actual sensor rendering',font=font(20,True),fill=TEXT)
        valid=np.isfinite(depth);color=cv2.applyColorMap(np.uint8(np.nan_to_num(np.clip(depth/6,0,1))*255),cv2.COLORMAP_TURBO)
        color[~valid]=(10,10,10)
        canvas.paste(Image.fromarray(color[:,:,::-1]).resize((600,250)),(1296,550))
        d.text((1302,520),'DEPTH / 0.35–6m / black = invalid',font=font(20,True),fill=TEXT)
        view=node.policy.view
        if view is not None:
            states=np.flipud(view.states.T);pixels=np.full((*states.shape,3),(28,47,65),np.uint8)
            pixels[states==0]=(40,128,119);pixels[states==1]=(234,133,103)
            pixels[states==2]=(155,109,212)
            tile=Image.fromarray(pixels).resize((240,240),Image.Resampling.NEAREST);draw=ImageDraw.Draw(tile)
            grid=view.planner.grid
            def uv(p):return ((p[0]-grid.origin[0])/grid.res/states.shape[1]*240,
                                240-(p[1]-grid.origin[1])/grid.res/states.shape[0]*240)
            if node.policy.gate is not None:
                path=node.policy.gate.follower.traj.p
                draw.line([uv(p) for p in path],fill=CYAN,width=2)
            x,y=uv(position);draw.ellipse((x-3,y-3,x+3,y+3),fill=TEXT)
            draw.line([(x,y),(x+12*math.cos(yaw),y-12*math.sin(yaw))],fill=TEXT,width=2)
            canvas.paste(tile,(1296,830))
        d.text((1550,834),'LIVE MAP',font=font(22,True),fill=TEXT)
        for j,(label,color) in enumerate([('Unknown / blocked',MUTED),('Observed free',(40,128,119)),('Occupied',(234,133,103)),('Launch certificate',(155,109,212)),('A* + clothoid path',CYAN)]):
            d.text((1550,870+j*34),label,font=font(20),fill=color)
        d.text((28,950),f't={t:06.1f}s   {node.policy.state}   replans={node.policy.replans}',font=font(27,True),fill=CYAN)
        d.text((28,1001),f'Distance {distance:.2f}m   clearance {minimum:.2f}m   σpose={node.pos_std:.3f}m',font=font(24),fill=TEXT)
        d.text((28,1042),'Forward depth only → team voxel map → existing A* / clothoid → follower → independent safety gate',font=font(21),fill=MUTED)
        self.writer.send(np.asarray(canvas))
        if self.count%100==0:canvas.save(self.out/'screenshots'/f'{self.count:06d}.png')
        self.count+=1

    def close(self):
        self.writer.close();self.renderer.close()
