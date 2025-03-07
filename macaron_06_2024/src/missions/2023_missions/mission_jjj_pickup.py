#0619 지금 안 쓰는 파일    
    
# def target_sign(self):
    #     if self.boxes[0][1] == 'delivery_a1':
    #         target = 'delivery_b1'

    #         print('target b 표지판 : ', target)

    #         return target

    #     elif self.boxes[0][1] == 'delivery_a2':
    #         target = 'delivery_b2'
    #         print('target b 표지판 : ', target)

    #         return target

    #     else:
    #         target = 'delivery_b3'
    #         print('target b 표지판 : ', target)

    #         return target


    # def stop_mission(self,data):
    #     self.boxes = list()
    #     for sign in data.obj:
    #         if sign.ns in self.sign_list:
    #             self.boxes.append(((abs(sign.xmin - sign.xmax)) * (abs(sign.ymin - sign.ymax)), sign.ns))

    #     endProcessArea = self.boxes[0]

    #     if endProcessArea > 5:
    #         return endProcessArea