"""Judge dashboard from actual recorded SIL data and labelled Blender replays."""
import argparse
import json
from pathlib import Path
import math
from functools import lru_cache
import numpy as np
from PIL import Image,ImageDraw,ImageFont
import imageio_ffmpeg

W,H,FPS=1920,1080,20
BG=(12,22,36); PANEL=(20,35,52); TEXT=(232,240,245); MUTED=(151,175,193)
CYAN=(46,215,199); AMBER=(255,191,90); RED=(241,104,86)
FONTS=Path('C:/Windows/Fonts')
@lru_cache(maxsize=128)
def font(size,bold=False,mono=False):
    return ImageFont.truetype(str(FONTS/('consola.ttf' if mono else ('seguisb.ttf' if bold else 'segoeui.ttf'))),size)


class Dashboard:
    def __init__(self,demo):
        self.demo=demo;self.cache={}
        self.traces={scene:json.loads((demo/'evidence'/scene/'trace.json').read_text()) for scene in ('course','diagnostic')}
        self.results={scene:json.loads((demo/'evidence'/scene/'result.json').read_text()) for scene in ('course','diagnostic')}

    def image(self,path):
        path=str(path)
        if path not in self.cache:self.cache[path]=Image.open(path).convert('RGB')
        return self.cache[path]

    def map(self,scene,index,size=286):
        data=np.load(self.demo/'evidence'/scene/f'{index:04d}.npz')
        grid=np.flipud(data['grid'].T);ny,nx=grid.shape
        rgb=np.full((*grid.shape,3),(28,47,65),np.uint8)
        rgb[grid==0]=(40,128,119);rgb[grid==1]=(234,133,103)
        image=Image.fromarray(rgb).resize((size,size),Image.Resampling.NEAREST);draw=ImageDraw.Draw(image)
        def uv(p):return ((p[0]-data['origin'][0])/data['resolution']/nx*size,
                         size-(p[1]-data['origin'][1])/data['resolution']/ny*size)
        plan=data['trajectory']
        if len(plan)>1:draw.line([uv(p) for p in plan],fill=(75,242,228),width=3)
        position=data['position'];x,y=uv(position);r=.46/data['resolution']/nx*size
        draw.ellipse((x-r,y-r,x+r,y+r),outline=AMBER,width=2)
        draw.ellipse((x-4,y-4,x+4,y+4),fill=TEXT)
        yaw=self.traces[scene][index]['yaw']
        draw.line([(x,y),(x+16*math.cos(yaw),y-16*math.sin(yaw))],fill=TEXT,width=3)
        return image

    @lru_cache(maxsize=8)
    def frame(self,scene,index):
        row=self.traces[scene][index];result=self.results[scene]
        canvas=Image.new('RGB',(W,H),BG);d=ImageDraw.Draw(canvas)
        d.text((36,25),'STANFORD DRONE HACKS',font=font(21,True),fill=CYAN)
        title='FORWARD CAMERA / ACTUAL COURSE' if scene=='course' else 'A* + CLOTHOID / LOCAL NAVIGATION'
        d.text((36,58),title,font=font(43,True),fill=TEXT)
        subtitle='Real STEP geometry. Startup is refused when clearance is unobserved.' if scene=='course' else 'Real policy components. Ideal panoramic coverage diagnostic — NOT OAK-D equivalence.'
        d.text((38,119),subtitle,font=font(23),fill=MUTED)
        d.rounded_rectangle((1450,30,1884,98),radius=14,fill=PANEL)
        d.text((1470,46),'RECORDED EXECUTION REPLAY',font=font(23,True),fill=AMBER)
        d.text((1455,116),f"t = {row['time']-1:05.1f} s  /  frame {index:03d}",font=font(22,mono=True),fill=MUTED)
        for box in [(28,174,1256,962),(1274,174,1892,590),(1274,606,1892,962)]:d.rounded_rectangle(box,radius=12,fill=PANEL)
        render_index=0 if scene=='course' else min(128,index//2*2)
        replay=self.image(self.demo/'replay_frames'/scene/f'{render_index:04d}.png').resize((1212,758),Image.Resampling.LANCZOS)
        canvas.paste(replay,(36,196))
        d=ImageDraw.Draw(canvas)
        d.rounded_rectangle((52,211,680,251),radius=8,fill=BG)
        d.text((66,218),'3D CUTAWAY • team airframe • recorded poses',font=font(20),fill=TEXT)
        if scene=='course':
            d.rounded_rectangle((62,857,1185,936),radius=10,fill=BG)
            d.text((82,867),'SAFETY HOLD — autonomous course navigation is NOT demonstrated',font=font(25,True),fill=AMBER)
            d.text((82,903),'Window / tunnel / blue-square geometry shown; traversal and landing not validated.',font=font(20),fill=MUTED)
        else:
            d.rounded_rectangle((62,860,1185,937),radius=10,fill=BG)
            label='NEW OBSTACLE → PATH VETO / REPLAN' if row['dynamic_obstacle'] else 'OBSERVED FREE SPACE → ACTUAL PLANNED TRAJECTORY'
            d.text((82,869),label,font=font(26,True),fill=CYAN)
            d.text((82,905),'Ideal 360° range input • kinematic SIL plant • no ROS DDS or PX4',font=font(21),fill=MUTED)
        d.text((1294,186),'FORWARD SENSOR VIEW',font=font(22,True),fill=TEXT)
        fpv=self.image(self.demo/'evidence'/scene/f'{index:04d}_fpv.png').resize((576,360),Image.Resampling.LANCZOS)
        canvas.paste(fpv,(1294,220))
        d=ImageDraw.Draw(canvas)
        d.rounded_rectangle((1305,535,1858,574),radius=6,fill=BG)
        d.text((1318,543),'Forward depth/RGB model' if scene=='course' else 'Display FPV; mapper receives ideal panorama',font=font(19),fill=AMBER)
        d.text((1294,617),'LIVE OBSERVED MAP',font=font(22,True),fill=TEXT)
        canvas.paste(self.map(scene,index),(1294,661));d=ImageDraw.Draw(canvas)
        for y,color,label in [(670,(28,47,65),'Unknown / blocked'),(708,(40,128,119),'Observed free'),(746,(234,133,103),'Occupied'),(784,CYAN,'Current A* path'),(822,AMBER,'Safety envelope')]:
            d.rounded_rectangle((1602,y,1620,y+18),radius=3,fill=color)
            d.text((1632,y-5),label,font=font(20),fill=MUTED)
        d.text((1602,868),'NO CAD INPUT',font=font(21,True),fill=CYAN)
        d.text((1602,900),'to the planner',font=font(20),fill=MUTED)
        speed=float(np.linalg.norm(row['command']))
        traveled=sum(np.linalg.norm(np.array(self.traces[scene][i]['position'])-self.traces[scene][i-1]['position']) for i in range(1,index+1))
        state=row['state'].replace('WAIT_MEASURED_FULL_START_VOLUME','START VOLUME UNKNOWN').replace('WAIT_FRESH_MAP','DEPTH LOST — HOLD')
        fields=[('POLICY STATE',state[:34]),('DISTANCE',f'{traveled:.2f} m'),('COMMAND SPEED',f'{speed:.2f} m/s'),('REPLANS',str(row['replans'])),('CONTACTS',str(result['collision_count']))]
        xs=[38,715,997,1360,1660]
        for (label,value),x in zip(fields,xs):
            d.text((x,978),label,font=font(16,True),fill=MUTED)
            d.text((x,1003),value,font=font(25,True),fill=AMBER if 'HOLD' in state or scene=='course' else TEXT)
        d.text((38,1053),'SOFTWARE ONLY   |   FULL COURSE: NOT COMPLETE   |   PHYSICAL FLIGHT READINESS: NOT CLAIMED',font=font(16),fill=MUTED)
        return canvas

    def card(self,heading,lines,tag='ENGINEERING EVIDENCE / EXPLICIT LIMITS'):
        im=Image.new('RGB',(W,H),BG);d=ImageDraw.Draw(im)
        for x in range(0,W,80):d.line([(x,0),(x,H)],fill=(16,29,44),width=1)
        for y in range(0,H,80):d.line([(0,y),(W,y)],fill=(16,29,44),width=1)
        d.rectangle((100,170,110,900),fill=CYAN)
        d.text((155,205),'STANFORD DRONE HACKS',font=font(27,True),fill=CYAN)
        d.text((155,285),heading,font=font(58,True),fill=TEXT)
        for i,line in enumerate(lines):d.text((158,420+i*70),line,font=font(29),fill=MUTED)
        d.text((158,886),tag,font=font(21,True),fill=AMBER)
        return im


def encode(path,frames):
    stream=imageio_ffmpeg.write_frames(str(path),(W,H),fps=FPS,codec='libx264',pix_fmt_in='rgb24',pix_fmt_out='yuv420p',
        quality=None,macro_block_size=1,output_params=['-crf','19','-preset','medium','-movflags','+faststart'])
    stream.send(None);count=0
    try:
        for image in frames:stream.send(np.array(image));count+=1
    finally:stream.close()
    return count


def export(demo):
    ui=Dashboard(demo);shots=demo/'screenshots';shots.mkdir(exist_ok=True)
    selections=[('01_course_and_safety_hold','course',30),('02_depth_loss_hold','course',75),
        ('03_live_mapping','diagnostic',20),('04_actual_planned_trajectory','diagnostic',32),
        ('05_obstacle_stop','diagnostic',40),('06_replanning','diagnostic',60),('07_local_navigation','diagnostic',128)]
    for name,scene,index in selections:ui.frame(scene,index).save(shots/(name+'.png'))
    title=ui.card('AUTONOMY WITHOUT A PRELOADED MAP',[
        'Real mapper. Real A* + clothoid. Real tracking and independent safety.',
        'Recorded local execution, visualized against the supplied course STEP.',
        'A truthful demonstration of both capability and the remaining blocker.'])
    transition=ui.card('NOW ISOLATE THE PLANNING STACK',[
        'The forward-camera course test correctly remains in startup HOLD.',
        'The next recording uses ideal panoramic coverage to isolate the policy.',
        'It is not a claim of OAK-D-equivalent sensing or full-course success.'])
    final=ui.card('VERIFIED COMPONENT BEHAVIOR',[
        '2.64 m local motion   •   5 replans   •   obstacle stop observed   •   0 contacts',
        'Forward-camera course result: HOLD, 0 m. Full course not completed.',
        'No ROS DDS / Gazebo / PX4 / hardware qualification is claimed.'])
    def main_frames():
        for _ in range(2*FPS):yield title
        for f in range(8*FPS):yield ui.frame('course',min(80,f//2))
        for _ in range(3*FPS):yield transition
        for f in range(13*FPS):yield ui.frame('diagnostic',min(129,f//2))
        for _ in range(4*FPS):yield final
    def highlight():
        intro=ui.card('STOP. REPLAN. CONTINUE.',[
            'Actual local navigation code, ideal panoramic sensor diagnostic.',
            'This clip does not demonstrate the physical forward-camera pipeline.'])
        for _ in range(2*FPS):yield intro
        for f in range(10*FPS):yield ui.frame('diagnostic',min(129,25+int(f*.5)))
        for _ in range(2*FPS):yield final
    manifest=dict(main_frames=encode(demo/'main_demo.mp4',main_frames()),
        highlight_frames=encode(demo/'highlight.mp4',highlight()),fps=FPS,resolution=[W,H],
        source='recorded execution traces + labelled Blender replay; not screen capture of ROS/Gazebo')
    (demo/'MEDIA_MANIFEST.json').write_text(json.dumps(manifest,indent=2));print(json.dumps(manifest))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--demo',type=Path,required=True)
    a=parser.parse_args();export(a.demo)
