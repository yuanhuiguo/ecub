using NamedPipeWrapper.Model;
using Newtonsoft.Json;
using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Runtime.InteropServices.ComTypes;
using System.Text;
using System.Threading;

namespace Eac.Uwb
{
    public class UWBSDK : IDisposable
    {
        public delegate void UwbDetailEventHandler(UwbDetail e);
        public event UwbDetailEventHandler DataReceived;

        public delegate void UsbDetaiulLogHandler(string e);
        public event UsbDetaiulLogHandler LogEvent;


        SerialPort serialPort1;
        public bool IsOpenCom()
        {
            return serialPort1?.IsOpen == true;
        }
        private int CRCL, CRCH, CRCBL, CRCBH;
        /// <summary>
        ///  串口接收数据缓存
        /// </summary>
        private static volatile byte[] _rBuff = null;
        /// <summary>
        /// 是否为新数据标志位
        /// </summary>
        private int GETDATA_NEW = 0;

        /// <summary>
        /// 测距发送协议使能
        /// </summary>
        private int CJ_EN = 0;
        /// <summary>
        ///  测距结束协议使能
        /// </summary>
        private int CJ_END = 0;

        private Timer checkDataTimer;

        private Timer checkStartOrStopTimer;

        private List<DeviceDetail> deviceDetails = new List<DeviceDetail>();

        /// <summary>
        /// 链接设备的ID号
        /// </summary>
        private byte NOW_ID = 1;
        private Timer EventTime;
        public UwbDetail uwbDetail;
        /// <summary>
        /// 计算实际距离的类
        /// </summary>

        private string _openComName;

        public int algStepDataTime { get; set; } = 500;

        /// <summary>
        /// 圈距离
        /// </summary>
        public float qsN { get; set; }
        /// <summary>
        /// //初始化测距SDK，可能会抛异常，设备找不到
        /// </summary>
        /// <param name="comName">Com口</param>
        /// <param name="number"></param>
        /// <param name="qsN">圈距离</param>
        /// <param name="checkStep">检测距离的频率（毫秒）</param>
        /// <param name="defaultTagDeviceCount">最低需要2个device</param>
        public UWBSDK(string comName, float qsN, AlgNumber number, int checkStep = 500, int defaultTagDeviceCount = 2)
        {
            this.qsN = qsN;
            uwbDetail = new UwbDetail(qsN, number);
            //绑定串口事件
            this.algStepDataTime = checkStep;
            serialPort1 = new SerialPort();
            serialPort1.DataReceived += new SerialDataReceivedEventHandler(SerDataReceive);
            checkDataTimer = new Timer(CheckDataTimer);
            checkStartOrStopTimer = new Timer(StartOrStopEvent);
            EventTime = new Timer(RangeTim_Tick);
            checkDataTimerStart();
            for (int i = 0; i < defaultTagDeviceCount; i++)
            {
                deviceDetails.Add(new DeviceDetail { TagId = i });
            }
            _openComName = comName;
        }

        private void RangeTim_Tick(object statu)
        {
            ComputeDistance();
            DataReceived(uwbDetail);
        }

        private void StartOrStopEvent(object state)
        {
            byte[] send_buf = new byte[100];
            if (CJ_EN == 1)
            {
                send_buf[0] = NOW_ID;
                send_buf[1] = 0x10;
                send_buf[2] = 0x00;
                send_buf[3] = 0x28;
                send_buf[4] = 0x00;
                send_buf[5] = 0x01;
                send_buf[6] = 0x02;
                send_buf[7] = 0x00;
                send_buf[8] = 0x04; //持续检测自动发送

                senddata(send_buf, 11);
            }
            else if (CJ_END == 1)
            {
                send_buf[0] = NOW_ID;
                send_buf[1] = 0x10;
                send_buf[2] = 0x00;
                send_buf[3] = 0x28;
                send_buf[4] = 0x00;
                send_buf[5] = 0x01;
                send_buf[6] = 0x02;
                send_buf[7] = 0x00;
                send_buf[8] = 0x00; //停止定位00         
                senddata(send_buf, 11);
            }
        }

        private void checkDataTimerStart()
        {
            checkDataTimer.Start(1);
        }

        private void checkStartOrStopTimerStart()
        {
            checkStartOrStopTimer.Start(200);
        }

        private void CheckDataTimer(object stat)
        {
            if (GETDATA_NEW == 1)//接收到新数据
            {

                GETDATA_NEW = 0;
                checkDataTimer.Stop(); //关闭自己时钟使能

                connect_Flag_Tick();


                checkDataTimerStart();//打开自己时钟使能
            }
        }

        bool isReciveDistant = false;

        /// <summary>
        /// 传输步数
        /// </summary>
        private void connect_Flag_Tick()
        {
            byte[] rBuff = new byte[_rBuff.Length];
            _rBuff.CopyTo(rBuff, 0);

            if (rBuff[1] == 0x10 && rBuff.Length == 8)  //收到协议应答
            {
                CJ_EN = 0;
                CJ_END = 0;
                checkStartOrStopTimer.Stop();
            }

            if (rBuff[1] == 0x03 &&
                      ((rBuff[2] == 0x28 && rBuff.Length == 45) ||
                      (rBuff[2] == 0x1A && rBuff.Length == 32)) //兼容以前的设备
                      )
            {
                lock (deviceDetails)
                {
                    foreach (var item in deviceDetails)
                    {
                        if (item.TagId == rBuff[4])
                        {
                            item.Distance = Convert.ToInt16((short)(((rBuff[13] << 8) & 0xFF00) | rBuff[14]));
                            //LogEvent($"{DateTime.Now:ss:fff}-{item.TagId}-{item.Distance}-{Environment.NewLine}");
                            isReciveDistant = true;
                            //LogHelper.Info($"isReciveDistant:{isReciveDistant}");
                            break;
                        }
                        LogHelper.Info($"deviceDetails TagId:{item.TagId},Distance:{item.Distance}");
                    }
                }
                if (rBuff[1] == 0x03 && rBuff.Length == 45)
                {
                    //如果重复就去掉
                    var acc_x = Convert.ToInt16((Int16)((rBuff[29] << 8) | rBuff[30])) / 32768.0 * 16;
                    var acc_y = Convert.ToInt16((Int16)((rBuff[31] << 8) | rBuff[32])) / 32768.0 * 16;
                    var acc_z = Convert.ToInt16((Int16)((rBuff[33] << 8) | rBuff[34])) / 32768.0 * 16;
                    LogHelper.Info($"acc_x:{acc_x}   acc_y:{acc_y} acc_z:{acc_z} stepData.StepCount:{uwbDetail.stepData.StepCount}");
                    uwbDetail.stepData.AddUWBStepXyz(new UWBStepXYZ { X = acc_x, Y = acc_y, Z = acc_z, DataRTime = DateTime.Now });
                }
                //LogEvent(string.Join(Environment.NewLine, deviceDetails.Select(m => $"{m.TagId}  {m.Distance}")));
            }
        }

        private int usRxLength = 0;

        private byte[] RxBuffer = new byte[200];

        private string ByteArrayToHexStr(byte[] byteArray, int length)
        {
            if (byteArray == null)
            {
                return null;
            }
            string hexString = BitConverter.ToString(byteArray, 0, length);

            return hexString.Replace('-', ' ');
        }

        /// <summary>
        /// 接收数据的串口事件
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void SerDataReceive(object sender, SerialDataReceivedEventArgs e)  //串口接收数据程序
        {
            if (!serialPort1.IsOpen)
            {
                return;
            }
            //Thread.Sleep(4);
            #region UWB12.2以前算法
            //byte[] byteReceive;  //串口数据接收数组
            //try
            //{
            //    byteReceive = new byte[serialPort1.BytesToRead];
            //}
            //catch
            //{
            //    return;
            //}


            //serialPort1.Read(byteReceive, 0, byteReceive.Length);

            //serialPort1.DiscardInBuffer();
            //if (byteReceive.Length < 5)
            //{
            //    LogHelper.Info($"SerDataReceive 数据丢弃");
            //    return;
            //}
            //var Text_Len = byteReceive.Length; //串口数据长度
            //int crc = byteReceive.CRC16(Text_Len - 2);
            //CRCH = crc / 256;
            //CRCL = crc % 256;
            //CRCBH = byteReceive[Text_Len - 2];
            //CRCBL = byteReceive[Text_Len - 1];

            //if (CRCBH != CRCH)
            //{
            //    LogHelper.Info("数据CRCH校验失败");
            //    return;
            //}

            //if (CRCBL != CRCL)
            //{
            //    LogHelper.Info("数据CRCL校验失败");
            //    return;
            //}


            //GETDATA_NEW = 1;   //标志接收到了数据
            //_rBuff = new byte[Text_Len];
            //Array.Copy(byteReceive, _rBuff, Text_Len); 
            #endregion

            try
            {
                ushort usLength = 0;
                try
                {
                    usLength = (ushort)serialPort1.Read(RxBuffer, usRxLength, RxBuffer.Length - usRxLength);
                }
                catch (Exception ex)
                {
                    LogHelper.Error("UWBSDK SerDataReceive start", ex);
                    return;
                }
                usRxLength += usLength;
                byte[] byteTemp = new byte[200];

                while (usRxLength >= 8)
                {
                    RxBuffer.CopyTo(byteTemp, 0);
                    //串口协议
                    if (byteTemp[0] != 0x01 || !(byteTemp[1] == 0x03 || byteTemp[1] == 0x06 || byteTemp[1] == 0x10))
                    {
                        //LogHelper.Info($"UWBSDK SerDataReceive  调整位置{ByteArrayToHexStr(byteTemp, usRxLength)}");
                        for (int i = 1; i < usRxLength; i++) RxBuffer[i - 1] = RxBuffer[i];
                        usRxLength--;
                        continue;
                    }
                    var Text_Len = 45;
                    if (byteTemp[1] == 0x06 || byteTemp[1] == 0x10)
                    {
                        Text_Len = 8;
                    }
                    if (usRxLength < Text_Len)
                    {
                        break;
                    }
                    int crc = byteTemp.CRC16(Text_Len - 2);
                    CRCH = crc / 256;
                    CRCL = crc % 256;
                    CRCBH = byteTemp[Text_Len - 2];
                    CRCBL = byteTemp[Text_Len - 1];
                    if (CRCBH != CRCH || CRCBL != CRCL)
                    {
                        LogHelper.Info("数据校验失败");
                        for (int i = 1; i < usRxLength; i++) RxBuffer[i - 1] = RxBuffer[i];
                        usRxLength--;
                        continue;
                    }

                    GETDATA_NEW = 1;   //标志接收到了数据
                    _rBuff = new byte[Text_Len];
                    Array.Copy(byteTemp, _rBuff, Text_Len);
                    for (int i = Text_Len; i < usRxLength; i++)
                        RxBuffer[i - Text_Len] = RxBuffer[i];
                    usRxLength -= Text_Len;
                }
            }
            catch (Exception ex)
            {
                LogHelper.Error("SerDataReceive ", ex);
            }
        }

        private void CloseAllDeviceSend()
        {
            for (int i = 0; i < 10; i++)
            {
                byte[] send_buf = new byte[300];
                send_buf[0] = NOW_ID;
                send_buf[1] = 0x06;
                send_buf[2] = 0x00;
                send_buf[3] = 0x28;
                send_buf[4] = 0x00;
                send_buf[5] = 0x00;
                send_buf[6] = 0x00;
                send_buf[7] = 0x00;
                senddata(send_buf, 8);
                Thread.Sleep(5);
            }
        }
        private void senddata(byte[] Frame, int Length)     // 串口发送数据    
        {
            try
            {
                Int32 crc;
                crc = CRCHelper.CRC16(Frame, Length - 2);    //进行CRC校验码的计算
                Frame[Length - 2] = (byte)(crc / 256);
                Frame[Length - 1] = (byte)(crc % 256);      //将计算得到的CRC校验码一起写入发送数据
                if (serialPort1.IsOpen)
                {
                    try
                    {
                        serialPort1.Write(Frame, 0, Length);
                    }
                    catch (Exception ex)
                    {
                        //记录日志
                        LogEvent(ex.StackTrace);
                    }
                }
            }
            catch (Exception ex)
            {
                LogHelper.Error("senddata", ex);
            }
        }

        public PatientInfo GetPatientInfo(string info)
        {
            try
            {
                return DeserializeJsonToObject<PatientInfo>(info);
            }
            catch (Exception ex)
            {
                LogHelper.Error("SetPatientInfo JsonToObject", ex);
                return null;
            }
        }

        private static T DeserializeJsonToObject<T>(string json) where T : class
        {
            JsonSerializer serializer = new JsonSerializer();
            StringReader sr = new StringReader(json);
            object o = serializer.Deserialize(new JsonTextReader(sr), typeof(T));
            T t = o as T;
            return t;
        }

        /// <summary>
        /// 检测圈数 实时距离
        /// </summary>
        private void ComputeDistance()
        {
            if (!isReciveDistant) return;

            //float minTagDis = qsN * 0.1f; //距离基站的位置
            int minTagDis = 200; //距离基站的位置
            lock (deviceDetails)
            {
                var first = deviceDetails.FirstOrDefault();
                var last = deviceDetails.LastOrDefault();
                LogHelper.Info($"ComputeDistance first:TagId:{first.TagId},Distance:{first.Distance}  last:TagId:{last.TagId},Distance:{last.Distance}");
                //如果为计圈数的算法
                if (uwbDetail.algNumber == AlgNumber.NumberOfTurns)
                {
                    //初始化
                    if (uwbDetail.CTag == -1)
                    {
                        if (Math.Abs((int)first.Distance) < minTagDis)
                        {
                            uwbDetail.CTag = first.TagId;
                            uwbDetail.RevertPoint();
                        }
                        else if (Math.Abs((int)last.Distance) < minTagDis)
                        {
                            uwbDetail.CTag = last.TagId;
                            uwbDetail.RevertPoint();
                        }
                    }
                    else
                    {
                        // 0 1是桩标号 -1是设备
                        #region 2023.2.14 范围计圈测距
                        var isADDGroup = false;
                        //当不为当前标签地点时 计算是否需要加1圈
                        if (first.TagId == uwbDetail.CTag)
                        {
                            if (Math.Abs(first.Distance) >= this.qsN * 0.94)
                            {
                                uwbDetail.CTag = last.TagId;
                                isADDGroup = true;
                            }
                            uwbDetail.OneGroupDictance = first.Distance;
                        }
                        else if (last.TagId == uwbDetail.CTag)
                        {
                            if (Math.Abs(last.Distance) >= this.qsN * 0.94)
                            {
                                uwbDetail.CTag = first.TagId;
                                isADDGroup = true;
                            }
                            uwbDetail.OneGroupDictance = last.Distance;
                        }
                        if (isADDGroup)
                        {
                            uwbDetail.GroupCount += 1;
                            uwbDetail.OneGroupDictance = 0;
                            return;
                        }
                        #endregion

                        
                    }
                }
                //如果为实际距离的算法
                else
                {
                    var current = new UWBPoint { X = first.Distance, Y = last.Distance };
                    var moveCm = uwbDetail.ActualUWBClass.CalculateDistance(current);
                    if (moveCm > 0) uwbDetail.OneGroupDictance += moveCm;
                }
            }
        }
        private bool isStart = false;

        /// <summary>
        /// 开始测距
        /// </summary>
        public void StartRanging()
        {
            if (isStart)
            {
                return;
            }
            CJ_EN = 1;
            checkStartOrStopTimerStart();
            uwbDetail.RevertPoint();
            EventTime.Start(algStepDataTime);
        }

        /// <summary>
        /// 开始测步数
        /// </summary>
        public void StartStep()
        {
            if (isStart)
            {
                return;
            }
            uwbDetail.stepData.RevertData();
        }

        public void Revert()
        {
            uwbDetail.RevertPoint();
        }
        public void StopRanging()
        {
            if (!isStart)
            {
                return;
            }
            CJ_END = 1;
            checkStartOrStopTimerStart();
            EventTime.Stop();
        }

        public void Init()
        {
            serialPort1.PortName = _openComName;
            serialPort1.BaudRate = 115200;
            serialPort1.DataBits = 8;
            serialPort1.ReceivedBytesThreshold = 1;
            serialPort1.StopBits = StopBits.One;
            serialPort1.Parity = Parity.None;
            serialPort1.ReadTimeout = 1000;
            serialPort1.WriteTimeout = 1000;
            serialPort1.Open();
            //LogHelper.Info($"UWBSDK Init PortName:{serialPort1.PortName} BaudRate:{serialPort1.BaudRate} IsOpen:{serialPort1.IsOpen}");
            CloseAllDeviceSend();
        }

        ~UWBSDK()
        {
            Dispose();
        }
        private void CloseSerialPort()
        {
            try
            {
                serialPort1?.Close();
                serialPort1 = null;
            }
            catch (Exception ex)
            {
                LogEvent(ex.StackTrace);
                return;
            }
        }

        public void Dispose()
        {
            try
            {
                CloseAllDeviceSend();
            }
            catch (Exception ex)
            {
                LogHelper.Error("Dispose close uwb error", ex);
            }
            try
            {
                CloseSerialPort();
                EventTime?.Dispose();
                checkDataTimer?.Dispose();
                checkStartOrStopTimer?.Dispose();
            }
            catch (Exception ex)
            {
                LogHelper.Error("Dispose close CloseSerialPort error", ex);
                //LogEvent(ex.StackTrace);
            }
        }
    }

    public class UWBStepXYZ
    {
        public DateTime DataRTime { get; set; }
        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }
    }

    public class UWBStepData
    {
        private double[] avg(double[] result)
        {
            int n = 5;
            double sum = 0;
            var b = new double[result.Length];
            for (var i = 0; i < result.Length; i++)
            {
                sum += result[i];
                if (i >= n) sum -= result[i - n];
                if (i >= n - 1) b[i] = sum / n;
            }
            return b;
        }

        private double[] Det(double[] data, int minStepHz = 11)
        {
            data = avg(data);
            var value = new double[data.Length];

            if (data.Length == 0) return value;

            double max = data[0];
            int lastMaxIndex = 0;
            //var avg = CalculateMean(data);// data.Average();
            for (int i = 1; i < data.Length; i++)
            {
                if (i - lastMaxIndex > minStepHz)
                {
                    value[lastMaxIndex] = max;
                    max = data[i];
                    lastMaxIndex = i;
                }

                if (max < data[i])
                {
                    max = data[i];
                    lastMaxIndex = i;
                }
            }
            if (lastMaxIndex != data.Length - 1)
            {
                value[lastMaxIndex] = max;
            }
            //去除低运动区间
            var min_value = 1.2;
            for (int i = 0; i < value.Length; i++)
            {
                if (value[i] < min_value)
                {
                    value[i] = 0;
                }
            }
            //去除独立点

            int min_step = 50;

            var lastIndex = -1;
            var lastLastIndex = -1;

            for (int i = 0; i < value.Length; i++)
            {
                if (value[i] > 0)
                {
                    if (lastIndex == -1)
                    {
                        lastIndex = i;
                        continue;
                    }
                    if (lastLastIndex != -1 && lastIndex != -1 && i - lastIndex > min_step && lastIndex - lastLastIndex > min_step)
                    {
                        value[lastIndex] = 0;
                        lastIndex = i;
                        continue;
                    }
                    else
                    {
                        lastLastIndex = lastIndex;
                        lastIndex = i;
                    }
                }
            }

            return value;
        }

        private static double[] oneDiff(double[] data)
        {
            double[] result = new double[data.Length - 1];
            for (int i = 0; i < result.Length; i++)
            {
                result[i] = data[i + 1] - data[i];
            }

            return result;
        }

        private static int[] trendSign(double[] data)
        {
            int[] sign = new int[data.Length];
            for (int i = 0; i < sign.Length; i++)
            {
                if (data[i] > 0) sign[i] = 1;
                else if (data[i] == 0) sign[i] = 0;
                else sign[i] = -1;
            }

            for (int i = sign.Length - 1; i >= 0; i--)
            {
                if (sign[i] == 0 && i == sign.Length - 1)
                {
                    sign[i] = 1;
                }
                else if (sign[i] == 0)
                {
                    if (sign[i + 1] >= 0)
                    {
                        sign[i] = 1;
                    }
                    else
                    {
                        sign[i] = -1;
                    }
                }
            }
            return sign;
        }

        private static List<int> getPeaksIndex(int[] diff)
        {
            List<int> data = new List<int>();
            for (int i = 0; i != diff.Length - 1; i++)
            {
                if (diff[i + 1] - diff[i] == -2)
                {
                    data.Add(i + 1);
                }
            }
            return data;//相当于原数组的下标
        }

        private static double CalculateStdDev(double[] values)
        {
            double ret = 0;
            if (values.Length > 0)
            {
                //  计算平均数   
                double avg = values.Average();
                //  计算各数值与平均数的差值的平方，然后求和 
                double sum = values.Sum(d => Math.Pow(d - avg, 2));
                //  除以数量，然后开方
                ret = Math.Sqrt(sum / values.Count());
            }
            return ret;
        }

        //private List<StepXYZ> stepXYZs = new List<StepXYZ>();
        List<double> vs = new List<double>();
        private Queue<double> tmpCalStep = new Queue<double>();
        private long removeLength = 0;
        //private long lastIndex = 0;
        //方差阈值
        private int left = 50;
        private int right = 50;
        private double[] devRange = new double[] { 0.2, 0.25, 0.35 };
        private double devCoff = 0d;
        private int removeLengthT = 0;
        private double mindevValue = 0.05d;
        public void AddUWBStepXyz(UWBStepXYZ s)
        {
            lock (this)
            {
                //stepXYZs.Add(s);
                var value = Math.Sqrt(Math.Pow(s.X, 2) + Math.Pow(s.Y, 2) + Math.Pow(s.Z, 2));
                vs.Add(value);
                tmpCalStep.Enqueue(value);
                #region 12.2以前算法
                //if (tmpCalStep.Count > 300)
                //{
                //    //新算法计算阈值 用到了均值滤波和波峰寻找
                //    var index2 = Det(tmpCalStep.ToArray());
                //    var start = left;
                //    if (removeLength == 0)
                //    {
                //        start = 0;
                //    }
                //    var cnt2 = index2.Skip(start).Take(tmpCalStep.Count - right - start).Count(m => m > 0);
                //    StepCount += cnt2;
                //    //int[] index = getPeaksIndex(trendSign(oneDiff(tmpCalStep.ToArray())));
                //    //index = index.Where(m => m >= start && m < tmpCalStep.Count - right).ToArray();
                //    //var result = new List<int>();
                //    //var last = 0;
                //    //foreach (var item in index)
                //    //{
                //    //    //波峰最低大于20hz
                //    //    if (item - last > 20)
                //    //    {
                //    //        StepCount++;
                //    //    }
                //    //    last = item;
                //    //}
                //    while (tmpCalStep.Count - left - right > 0)
                //    {
                //        tmpCalStep.Dequeue();
                //        removeLength++;
                //    }
                //}
                #endregion
                var queIndexs = new List<int>();
                if (tmpCalStep.Count > 200)
                {
                    var tmp = tmpCalStep.ToArray();
                    var ds = CalculateStdDev(tmp);
                    var avg = vs.Average();
                    devCoff = devRange[1];
                    //
                    var minValue = Math.Max(ds * devCoff, mindevValue);
                    for (int j = 0; j < tmp.Length; j++)
                    {
                        tmp[j] = tmp[j] - avg;
                        if (Math.Abs(tmp[j]) < minValue)
                        {
                            tmp[j] = 0;
                        }
                    }
                    var index = getPeaksIndex(trendSign(oneDiff(tmp)));
                    var start = left;
                    if (removeLengthT == 0)
                    {
                        start = 0;
                    }
                    var cntCount = index.Where(m => m >= start && m < tmp.Length - right).Count();
                    StepCount += cntCount;
                    queIndexs.AddRange(index.Where(m => m >= start && m < tmp.Length - right).Select(m => m + removeLengthT));
                    while (tmpCalStep.Count - left - right > 0)
                    {
                        tmpCalStep.Dequeue();
                        removeLengthT++;
                    }
                }
            }
        }

        public bool IsSuccess => tmpCalStep.Count > 0;
        public int StepCount { get; set; }

        public void RevertData()
        {
            StepCount = 0;
            tmpCalStep.Clear();
        }
    }

    public class UwbDetail : EventArgs
    {
        /// <summary>
        /// 当前选择的计步算法类型
        /// </summary>
        public AlgNumber algNumber { get; private set; }
        public UWBStepData stepData { get; set; }
        public bool IsInitSuccess
        {
            get
            {
                //return ActualUWBClass?.LastPoint != null;
                return TotalDictance > 0;
            }
        }
        public double Qsn { get; set; }

        //private double diffGroup = 100;//初始化绕圈距离1米
        //private double diffGroup = 300;//初始化绕圈距离1米

        /// <summary>
        /// 初始化最小的桩距
        /// </summary>  
        //public double MinGroupJuli = 200;
        //public double MinGroupJuli = 150;
        public UwbDetail(double qsN, AlgNumber alg)
        {
            this.Qsn = qsN;
            //MinGroupJuli = qsN * 0.1;
            //this.Qsn = qsN + diffGroup;//每圈要加一个绕圈距离
            ActualUWBClass = new ActualUWBClass(qsN);
            stepData = new UWBStepData();
            this.algNumber = alg;
        }
        /// <summary>
        /// 圈数
        /// </summary>
        public int GroupCount { get; set; }
        /// <summary>
        /// 当前距离单位厘米 （CM）
        /// </summary>
        public double OneGroupDictance { get; set; }
        /// <summary>
        /// 当前定位的标签ID
        /// </summary>
        public int CTag { get; set; } = -1;

        public ActualUWBClass ActualUWBClass { get; private set; }
        /// <summary>
        /// 显示用的距离
        /// </summary>
        public double viewDictance;

        /// <summary>
        /// 记录临时的总距离变量 防止距离缩减
        /// </summary>
        private double _totalDictance;
        /// <summary>
        /// 实际距离厘米
        /// </summary>
        public double TotalDictance
        {
            get
            {
                var BuJu = OneGroupDictance;
                if (algNumber == AlgNumber.NumberOfTurns)
                {
                    BuJu = GroupCount * Qsn + OneGroupDictance;
                    //BuJu = GroupCount * (Qsn + 300) + OneGroupDictance;
                }
                if (_totalDictance < BuJu) _totalDictance = BuJu;
                return _totalDictance;
            }
        }
        private bool IsStart = false;
        private bool IsEnd = false;
        private bool addExtraDistance = false;

        public void EndGroup()
        {
            if (!IsEnd)
            {
                IsEnd = true;
                IsStart = false;
            }
        }

        public void StartGroup()
        {
            if (!IsStart)
            {
                //新增一圈
                if (IsEnd) GroupCount++;
                //OneGroupDictance = MinGroupJuli;
                IsStart = true;
                IsEnd = false;
            }
        }

        public void AddMove(double move)
        {
            //    if (!IsStart)
            //    {
            //        OneGroupDictance += move;
            //    }

            if (addExtraDistance)
            {
                OneGroupDictance += 300;
                addExtraDistance = false;
            }
            OneGroupDictance += move;
        }

        public void AddMoveEnd()
        {
            addExtraDistance = true;
        }

        /// <summary>
        /// 厘米每秒
        /// </summary>
        public double Speed
        {
            get
            {
                return TotalDictance / (DateTime.Now - DtNow).TotalSeconds;
            }
        }

        public DateTime DtNow { get; private set; }
        public void RevertPoint()
        {
            ActualUWBClass.RevertPoint();
            OneGroupDictance = 0;
            GroupCount = 0;
            DtNow = DateTime.Now;
        }
    }

    public class DeviceDetail
    {
        public int TagId { get; set; }
        private short _distance = short.MinValue;
        public short Distance
        {
            get { return _distance; }
            set
            {
                _distance = value;
            }
        }
    }

    public class UWBPoint
    {
        public DateTime Time { get; private set; }
        public UWBPoint()
        {
            Time = DateTime.Now;
        }
        public double X { get; set; }
        public double Y { get; set; }
    }

    public class ActualUWBClass
    {
        public double Qsn { get; set; }
        public ActualUWBClass(double qsN)
        {
            this.Qsn = qsN;
        }
        public UWBPoint LastPoint { get; set; }
        int minStepDis = 50;//30cm

        public double InitDis { get; set; } = 0;
        public double CalculateDistance(UWBPoint currentPoint)
        {
            LogHelper.Info($"CalculateDistance currentPointX:{currentPoint.X} currentPointY:{currentPoint.Y}");
            currentPoint.X = GetDistance(currentPoint.X);
            currentPoint.Y = GetDistance(currentPoint.Y);
            double move = 0;
            var successValue = short.MaxValue / 2;
            if (currentPoint == null || Math.Abs(currentPoint.X) > successValue || Math.Abs(currentPoint.Y) > successValue) return move;
            if (LastPoint != null)
            {
                var mv1 = (Math.Pow(currentPoint.X, 2) - Math.Pow(currentPoint.Y, 2) + Math.Pow(Qsn, 2)) / (Qsn * 2);
                var mv2 = (Math.Pow(LastPoint.X, 2) - Math.Pow(LastPoint.Y, 2) + Math.Pow(Qsn, 2)) / (Qsn * 2);
                move = Math.Abs(mv1 - mv2);
            }
            else
            {
                InitDis = Math.Min(currentPoint.X, currentPoint.Y);
                LastPoint = currentPoint;
            }
            if (move > minStepDis)
            {
                LastPoint = currentPoint;
                return move;
            }
            return 0d;
        }

        public double GetDistance(double x)
        {
            return x - InitDis * (this.Qsn - x) / this.Qsn;
        }

        public void RevertPoint()
        {
            LastPoint = null;
        }
    }

}
