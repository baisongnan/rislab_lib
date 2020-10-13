import time
import scipy.io as sio  # save data as a .mat file


class DataSaver(object):
    def __init__(self, *args):
        self.varNameList = args
        self.varNum = len(args)
        self.info_list = {'other_information': -1}
        for i in range(self.varNum):
            if i == 0:
                self.varList = ([],)
            else:
                self.varList = self.varList + ([],)

    def add_elements(self,*args):
        if len(args) == self.varNum:
            for i in range(len(args)):
                self.varList[i].append(args[i])
        else:
            print('element number error')

    def add_info(self, info_dict):
        self.info_list.update(info_dict)

    def save2mat(self,save_path):
        time_temp = time.strftime('%Y%m%d_%H%M%S', time.localtime(time.time()))
        save_fnt = save_path + time_temp + '.mat'
        data_dict = {'exptime': time_temp, }
        for i in range(self.varNum):
            data_dict.update({self.varNameList[i]: self.varList[i]})
        data_dict.update(self.info_list)
        sio.savemat(save_fnt, data_dict)
        print('Data saved: ' + save_fnt)

    def save2mat_tail(self, save_path, tail):
        time_temp = time.strftime('%Y%m%d_%H%M%S', time.localtime(time.time()))
        save_fnt = save_path + time_temp + tail + '.mat'
        data_dict = {'exptime': time_temp, }
        for i in range(self.varNum):
            data_dict.update({self.varNameList[i]: self.varList[i]})
        data_dict.update(self.info_list)
        sio.savemat(save_fnt, data_dict)
        print('Data saved: ' + save_fnt)




