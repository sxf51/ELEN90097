# -*- coding: utf-8 -*-
from manim import *


class LDI_1(Scene):
    def construct(self):

        tit = Text("ELEN90097 LDI Project")
        tit.shift(UP * 1.5)
        self.play(Write(tit), run_time=3)

        t = Text("Xiufu SUN, Zhihan XU").next_to(tit, DOWN * 2)
        t1 = Text("Quadcopter unmanned aerial vehicle: ", font_size=30).next_to(t, DOWN * 3)
        t2 = Text("mathematical modeling, simulation, data storage and analysis", font_size=30).next_to(t1, DOWN * 0.5)
        self.play(Write(t), run_time=2)
        self.play(Write(t1), run_time=1)
        self.play(Write(t2), run_time=3)
        
        self.wait()

class LDI_2(Scene):
    def construct(self):
        
        t = Text("Quadcopter State Space", font_size=30).shift(UP * 2)
        tex = VGroup(
            Tex(r"$\dot{\xi} = f(\xi, w) = \begin{bmatrix} \
                        S_b^w v_b \\ \
                        \frac{1}{2} q \bigotimes \omega \\ \
                        \frac{1}{m} (F_b + F_d) - {S_b^w}^T g \textbf{1}_z - \omega \times v_b \\ \
                        I^{-1} (M_b - \omega \times I \omega) \
                    \end{bmatrix}$"),
            )
        tex.arrange(RIGHT, buff=SMALL_BUFF)
        tex.next_to(t, DOWN * 3)
        self.play(Write(t), run_time=2)
        self.play(Write(tex), run_time=5)

        self.wait()


class LDI_3(Scene):
    def construct(self):
        title = Text("13 States", font_size=40).to_edge(UP)
        
        LEFT_MARGIN = 2.5  
        def create_label(text):
            return (
                Text(text, font_size=28)
                .align_to(LEFT_MARGIN * LEFT, LEFT)
                .set_alignment(RIGHT)
            )
        
        entries = [
            ("Position:", r"$p = (x, y, z)^T$"),
            ("Attitude(Quaternion):", r"$q = (q_w, q_x, q_y, q_z)^T$"),
            ("Velocity:", r"$\mathbf{v}_b = (v_x, v_y, v_z)^T$"),
            ("Angular velocity:", r"$\omega = (\omega_x, \omega_y, \omega_z)^T$")
        ]
        
        group = VGroup()
        for label_text, formula in entries:
            label = create_label(label_text)
            equation = Tex(formula, font_size=28).next_to(label, RIGHT, buff=0.3)
            entry = VGroup(label, equation)
            group.add(entry)
        
        group.arrange(DOWN, aligned_edge=LEFT, buff=0.5)
        group.next_to(title, DOWN, buff=1.0)

        self.play(Write(title), run_time=2)
        self.play(
            LaggedStart(*[
                FadeIn(entry, shift=0.5*RIGHT) 
                for entry in group
            ], lag_ratio=0.3),
            run_time=6
        )
        self.wait()

class PlaneRotation(ThreeDScene):
    def construct(self):
        self.set_camera_orientation(phi=75 * DEGREES, theta=-45 * DEGREES)
        
        plane = Rectangle(width=3, height=3, color=BLUE, fill_opacity=0.5)
        #plane.rotate(PI/2, RIGHT)
        
        axes = ThreeDAxes()
        self.add(axes, plane)
        
        self.play(
            Rotate(plane, 15*DEGREES, axis=RIGHT),
            run_time=2
        )

        self.wait(1)
        
        plane_frame = plane.copy()
        plane_frame.set_opacity(0)
        
        plane_frame.rotate(15*DEGREES, RIGHT)
        
        y_axis = plane_frame.get_top() - plane_frame.get_center()
        y_axis = y_axis / np.linalg.norm(y_axis)
        
        self.play(
            Rotate(plane, 15*DEGREES, axis=y_axis),
            run_time=2
        )
        self.wait(2)


class LDI_4(Scene):
    def construct(self):
        
        t = Text("Quaternion", font_size=30).shift(UP * 2)
        tex = VGroup(
            Tex(r"$q = \begin{bmatrix}\
                q_w \\\
                q_x \\\
                q_y \\\
                q_z\
            \end{bmatrix} = \begin{bmatrix}\
                cos(\theta/2) \\\
                sin(\theta/2)r_{x} \\ \
                sin(\theta/2)r_{y} \\ \
                sin(\theta/2)r_{z}\
            \end{bmatrix}$"),
            )
        tex.arrange(RIGHT, buff=SMALL_BUFF)
        tex.next_to(t, DOWN * 3)
        self.play(Write(t), run_time=1)
        self.play(Write(tex), run_time=3)

        self.wait()


class LDI_5(Scene):
    def construct(self):

        tex1 = VGroup(
            Tex(r"$q_1 = \begin{bmatrix}\
                    cos(5\pi/180) \\\
                    sin(5\pi/180) \times 1 \\\
                    sin(5\pi/180) \times 0 \\\
                    sin(5\pi/180) \times 0\
                \end{bmatrix} \approx \begin{bmatrix}\
                    0.996195 \\\
                    0.087156 \\\
                    0 \\\
                    0\
                \end{bmatrix}$"),
            )
        tex2 = VGroup(
            Tex(r"$q_2 = \begin{bmatrix}\
                    cos(5\pi/180) \\\
                    sin(5\pi/180) \times 0 \\\
                    sin(5\pi/180) \times 1 \\\
                    sin(5\pi/180) \times 0\
                \end{bmatrix} \approx \begin{bmatrix}\
                    0.996195 \\\
                    0 \\\
                    0.087156 \\\
                    0\
                \end{bmatrix}$"),
            )
        tex1.arrange(RIGHT, buff=SMALL_BUFF)
        tex1.shift(UP)
        tex2.arrange(RIGHT, buff=SMALL_BUFF)
        tex2.next_to(tex1, DOWN * 2)
        self.play(Write(tex1), run_time=1)
        self.play(Write(tex2), run_time=1)
        self.wait(2)
        self.play(Unwrite(tex1), run_time=0.5)
        self.play(Unwrite(tex2), run_time=0.5)

        tex = VGroup(
            Tex(r"$q_1 \bigotimes q_2 \approx \
                \begin{bmatrix}\
                    0.9924 \\\
                    0.0868 \\\
                    0.0868 \\\
                    0.0076\
                \end{bmatrix}$"),
            )
        self.play(Write(tex), run_time=1)

        self.wait(3)

class LDI_6(Scene):
    def construct(self):

        tex = VGroup(
            Tex(r"$S_b^w= \begin{bmatrix}\
                1-2q_y^2-2q_z^2  & 2(q_x\cdot q_y-q_w\cdot q_z) & 2(q_x\cdot q_z + q_w\cdot q_y) \\\
                2(q_x\cdot q_y + q_w\cdot q_z)  & 1-2q_x^2-2q_z^2 & 2(q_y\cdot q_z - q_w\cdot q_x)\\\
                2(q_x\cdot q_z - q_w\cdot q_y)  & 2(q_y\cdot q_z + q_w\cdot q_x) & 1-2q_x^2-2q_y^2\
            \end{bmatrix}$", font_size=30),
            )
        self.play(Write(tex), run_time=1)

        self.wait(5)


class LDI_7(Scene):
    def construct(self):

        tex = VGroup(
            Tex(r"$\dot{q} = \frac{1}{2} q \bigotimes \begin{bmatrix}\
                    0 \\ \mathbf{\omega}\
                \end{bmatrix} = \frac{1}{2}\begin{bmatrix}\
                q_w & -q_x & -q_y & -q_z \\\
                q_x & q_w & -q_z & q_y \\\
                q_y & q_z & q_w & -q_x \\\
                q_z & -q_y & q_x & q_w\
            \end{bmatrix} \begin{bmatrix}\
                0 \\\
                \omega_x \\\
                \omega_y \\\
                \omega_z\
            \end{bmatrix}$", font_size=30),
            )
        self.play(Write(tex), run_time=2)

        self.wait(3)


class LDI_8(Scene):
    def construct(self):

        tex = VGroup(
            Tex(r"$\dot{p} = S_b^w v_b$"),
            )
        self.play(Write(tex), run_time=1)

        self.wait(5)

class LDI_9(Scene):
    def construct(self):

        tex = VGroup(
            Tex(r"$\dot{v}_b = \frac{1}{m}(F_b + F_d) - {S_b^w}^T g \mathbf{1}_z - \omega \times v_b$"),
            Tex(r"$F_b = C_t\cdot \sum w^2$"),
            Tex(r"$F_d = -K_d \|v_b\| v_b$"),
            ).shift(UP * 3)
        tex.arrange(DOWN * 1.5, buff=SMALL_BUFF)
        self.play(Write(tex), run_time=2)

        self.wait(3)

class LDI_10(Scene):
    def construct(self):

        tex1 = VGroup(
            Tex(r"$\mathrm {M}=\mathrm{I}\dot{\omega} + \omega\times(\mathrm{I} \cdot \omega) = C_d\cdot \sum w^2$"),
            )
        self.play(Write(tex1), run_time=2)
        self.play(Unwrite(tex1), run_time=0.5)

        self.wait()

        tex = VGroup(
            Tex(r"$\dot{\omega} = \begin{bmatrix}\
                \dot{\omega}_x \\\
                \dot{\omega}_y \\\
                \dot{\omega}_z\
            \end{bmatrix} = \begin{bmatrix}\
                \frac{M_x + (I_{yy} - I_{zz})\omega_y\omega_z}{I_{xx}} \\\
                \frac{M_y + (I_{zz} - I_{xx})\omega_x\omega_z}{I_{yy}} \\\
                \frac{M_z + (I_{xx} - I_{yy})\omega_x\omega_y}{I_{zz}}\
            \end{bmatrix}$"),
            )
        self.play(Write(tex), run_time=2)

        self.wait(3)

class LDI_11(Scene):
    def construct(self):
        
        t = Text("The End").shift(UP)
        tt = Text("Thank you for watching", font_size=30)
        tt.next_to(t, DOWN*3)
        self.play(Write(t), run_time=2)
        self.play(Write(tt), run_time=2)
        self.wait(5)

with tempconfig({'quality':'medium_quality', 'preview':True}):
    scene = LDI_11()
    scene.render()
