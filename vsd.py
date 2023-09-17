from matplotlib import pyplot as plt
class VehicleSimpleDynamic:
    def __init__(self, anglePosition, angleVelocity, i_y, k_b, l, cmdT):
        self.anglePosition = anglePosition
        self.angleVelocity = angleVelocity
        self.i_y = i_y
        self.k_b = k_b
        self.l = l
        self.cmdT = cmdT
        self.angleAcceleration = 0

    def calculate(self, angleAccelerationCmd, dt):
        self.calculateAngleAcceleration(angleAccelerationCmd)
        self.calculateAngleVelocityAndPosition(dt)

    def calculateAngleAcceleration(self, angleAccelerationCmd):
        w_1 = angleAccelerationCmd + self.cmdT
        w_2 = -angleAccelerationCmd + self.cmdT
        m_y = self.k_b * self.l * (w_1 * w_1 - w_2 * w_2)
        self.angleAcceleration = m_y / self.i_y

    def calculateAngleVelocityAndPosition(self, dt):
        self.angleVelocity += self.angleAcceleration * dt
        self.anglePosition += self.angleVelocity * dt

    def getAnglePosition(self):
        return self.anglePosition

    def getAngleVelocity(self):
        return self.angleVelocity

    def getAngleAcceleration(self):
        return self.angleAcceleration


class ControlSystem:
    def __init__(self, anglePositionCmd, k_p, k_i, k_d):
        self.anglePositionCmd = anglePositionCmd
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d

        self.errorPosition = 0
        self.errorPositionPast = 0
        self.errorVelocity = 0
        self.errorVelocityPast = 0
        self.angleVelocityCmd = 0
        self.angleAccelerationCmd = 0
        self.integralVelocity = 0
        self.integralAcceleration = 0
        self.controlLimit = 600


    def calculateAngleVelocityCmd(self, currentAnglePosition, dt):
        self.errorPositionPast = self.errorPosition
        self.errorPosition = self.anglePositionCmd - currentAnglePosition
        self.integralVelocity += self.errorPosition * dt
        self.angleVelocityCmd = self.k_p * self.errorPosition + self.k_i * self.integralVelocity + self.k_d * (self.errorPosition - self.errorPositionPast) / dt
        self.angleVelocityCmd = self.saturation(self.angleVelocityCmd)


    def calculateAngleAccelerationCmd(self, currentAngleVelocity, dt):
        self.errorVelocityPast = self.errorVelocity
        self.errorVelocity = self.angleVelocityCmd - currentAngleVelocity
        self.integralAcceleration += self.errorVelocity * dt
        self.angleAccelerationCmd = self.k_p * self.errorVelocity + self.k_i * self.integralAcceleration + self.k_d * (self.errorVelocity - self.errorVelocityPast) / dt
        self.angleAccelerationCmd = self.saturation(self.angleAccelerationCmd)

    def getAngleAccelerationCmd(self, currentAnglePosition, currentAngleVelocity, dt):
        self.calculateAngleVelocityCmd(currentAnglePosition, dt)
        self.calculateAngleAccelerationCmd(currentAngleVelocity, dt)
        return self.angleAccelerationCmd

    def saturation(self, value):
        if value > self.controlLimit:
            value = self.controlLimit
        elif value < -self.controlLimit:
            value = -self.controlLimit
        return value

class Simulator():

    def __init__(self, Tend, dt, controlSys, dynamicModel):
        self.dt = dt
        self.Tend = Tend
        self.controlSys = controlSys
        self.dynamicModel = dynamicModel
        self.accList = []
        self.velList = []
        self.posList = []
        self.timeList = []

    def runSimulation(self):
        time = 0
        while (time <= self.Tend):
            pose = self.dynamicModel.getAnglePosition()
            vel = self.dynamicModel.getAngleVelocity()
            acc = self.dynamicModel.getAngleAcceleration()

            self.posList.append(pose)
            self.velList.append(vel)
            self.accList.append(acc)

            angleAccelerationCmd = self.controlSys.getAngleAccelerationCmd(pose, vel, self.dt)

            self.dynamicModel.calculate(angleAccelerationCmd, self.dt)

            time += self.dt

    def showPlots(self):
        f = plt.figure(constrained_layout=True)
        gs = f.add_gridspec(3, 5)
        ax1 = f.add_subplot(gs[0, :-1])
        ax1.plot(self.posList)
        ax1.grid()
        ax1.set_title('position')

        ax2 = f.add_subplot(gs[1, :-1])
        ax2.plot(self.velList, "g")
        ax2.grid()
        ax2.set_title('velocity')

        ax3 = f.add_subplot(gs[2, :-1])
        ax3.plot(self.accList, "r")
        ax3.grid()
        ax3.set_title('acceleration')

        plt.show()


'''
 Объявим параметры для моделирования
'''
k_p = 400 # коэффициент Пропорционального регулирования
k_i = 15  # коэффициент Интегрального регулирования
k_d = 280  # коэффициент Дифференциального регулирования
dt = 0.01  # шаг моделирования системы (например одна сотая секунды)

Tend = 20 # конечное время моделирования (например 20 сек)

# Масса ЛА
mass = 0.006
# Коэффициент тяги двигателя ЛА
k_b = 3.9865e-08
# Количество двигателей ЛА
anglePositionCmd = 30
# Ограничение на угловую скорость двигателей рад/сек
motorSpeedLimit = 1000

'''
Создадим объект контроллера и объект для нашей математической модели
'''
controller = ControlSystem(anglePositionCmd, k_p, k_i, k_d)
uavSimpleDynamic = VehicleSimpleDynamic(0, 0, 7.16914e-5, 3.9865e-8, 0.17, 10)

sim = Simulator(Tend, dt, controller, uavSimpleDynamic)
sim.runSimulation()
sim.showPlots()
