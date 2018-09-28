using System;
using System.Net.Sockets;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading;
using UnityEngine;
using UnityEngine.UI;
using UnityStandardAssets.CrossPlatformInput;

public class TCPClient : MonoBehaviour
{
    public string ThrottleAxisName = "Throttle";
    public string YawAxisName = "Yaw";
    public string RollAxisName = "Roll";
    public string PitchAxisName = "Pitch";
    public Text IPBox;
    public Text PortBox;
    public Text LogBox;
    public Text KpBox;
    public Text KiBox;
    public Text KdBox;
    public Toggle MotorEnable;
    public Toggle EscCalibration;
    public Toggle MpuCalibration;

    private float Throttle;
    private float Yaw;
    private float Roll;
    private float Pitch;

    private Regex IPReg = new Regex(@"\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}");
    private Regex PortReg = new Regex(@"\d{1,4}");
    private Regex KReg = new Regex(@"-?\d{1,5}\.\d{1,7}");

    #region private members 	
    private TcpClient socketConnection;
    private Thread clientReceiveThread;
    #endregion

    void Start()
    {
        GetAllAxis();
    }

    void Update()
    {
        if (!GetAllAxis())  //Если значение поменялось, отправляем сообщение
        {
            SendMessage(false);
        }
    }

    bool GetAllAxis()   //True если старое значение равно считываемому
    {
        bool output = (Throttle == CrossPlatformInputManager.GetAxis(ThrottleAxisName)) &
                      (Yaw == CrossPlatformInputManager.GetAxis(YawAxisName)) &
                      (Roll == CrossPlatformInputManager.GetAxis(RollAxisName)) &
                      (Pitch == CrossPlatformInputManager.GetAxis(PitchAxisName));
        if(!output)
        {
            Throttle = CrossPlatformInputManager.GetAxis(ThrottleAxisName);
            Yaw = CrossPlatformInputManager.GetAxis(YawAxisName);
            Roll = CrossPlatformInputManager.GetAxis(RollAxisName);
            Pitch = CrossPlatformInputManager.GetAxis(PitchAxisName);
        }
        return output;
    }

    string GetStringMessage(bool PID = false)
    {
        string output;
        if (!PID)
        {

            output = " {\"Type\": true,\"Enabled\": ";
            if (MotorEnable.isOn) output += "true";
            else output += "false";
            //output += ",\"Throttle\": " + (int)((Throttle + 3) * 500) + ",\"Yaw\": " + (int)((Yaw + 3) * 500) + ",\"Roll\": " + (int)((Roll + 3) * 500) + ",\"Pitch\": " + (int)((Pitch + 3) * 500) + '}';
            output += ",\"Controller\": [" + (int)((Yaw + 3) * 500) + "," + (int)((Pitch + 3) * 500) + "," + (int)((Roll + 3) * 500) + "," + (int)((Throttle + 3) * 500) + "]}";
            Debug.Log("Рысканье/Тангаж/Крен/Тяга: " + output);
            LogBox.text = "Рысканье/Тангаж/Крен/Тяга: " + output + "\n";
        }
        else
        {
            output = " {\"Type\": false,\"Enabled\": ";
            if (MotorEnable.isOn) output += "true,\"ESC\": ";
            else output += "false,\"ESC\": ";
            if (EscCalibration.isOn) output += "true,\"MPU\": ";
            else output += "false,\"MPU\": ";
            if (MpuCalibration.isOn) output += "true";
            else output += "false";
            //output += ",\"Kp\": " + KpBox.text + ",\"Ki\": " + KiBox.text + ",\"Kd\": " + KdBox.text + '}';
            output += ",\"K\": [" + KpBox.text + "," + KiBox.text + "," + KdBox.text + "]}";
            Debug.Log("Kp/Ki/Kd: " + output);
            LogBox.text = "Kp/Ki/Kd: " + output + "\n";
        }
        return output;
    }

    public void TryToConnect()  //Срабатывает от нажатия кнопки "Подключиться"
    {
        if (IPReg.IsMatch(IPBox.text) & PortReg.IsMatch(PortBox.text))  //Проверка на соответствие IP адреса
        {
            ConnectToTcpServer();
        }
        else
        {
            Debug.Log("Введен неправильный IP или Port");
            LogBox.text = "Введен неправильный IP или Port\n";
        }
    }

    public void TryToSendPID()  //Срабатывает от нажатия кнопки "Записать"
    {
        if (KReg.IsMatch(KpBox.text) & KReg.IsMatch(KiBox.text) & KReg.IsMatch(KdBox.text))
        {
            SendMessage(true);
        }
    }

    //Устанавливаем соединение
    private void ConnectToTcpServer()
    {
        try
        {
            clientReceiveThread = new Thread(new ThreadStart(ListenForData));
            clientReceiveThread.IsBackground = true;
            clientReceiveThread.Start();
            Debug.Log("Подключение установлено.");
            LogBox.text = "Подключение установлено.\n";
        }
        catch (Exception e)
        {
            Debug.Log("Вызвано исключение на сервере: " + e);
            LogBox.text = "Вызвано исключение на сервере: " + e + "\n";
        }
    }	

    //Работает на заднем плане для приема сообщений
    private void ListenForData()
    {
        try
        {
            socketConnection = new TcpClient(IPBox.text, int.Parse(PortBox.text));
            Byte[] bytes = new Byte[1024];
            while (true)
            {
                //Получаем стрим			
                using (NetworkStream stream = socketConnection.GetStream())
                {
                    int length;
                    //Читаем стрим в массив байт 					
                    while ((length = stream.Read(bytes, 0, bytes.Length)) != 0)
                    {
                        var incommingData = new byte[length];
                        Array.Copy(bytes, 0, incommingData, 0, length);
                        //Переводим массив байт в строку						
                        string serverMessage = Encoding.ASCII.GetString(incommingData);
                        Debug.Log("Получено сообщение: " + serverMessage);
                        LogBox.text = "Получено сообщение: " + serverMessage + "\n";
                    }
                }
            }
        }
        catch (SocketException socketException)
        {
            Debug.Log("Исключение на сокете: " + socketException);
            LogBox.text+= "Исключение на сокете: " + socketException + "\n";
        }
    }

    //Отправляем сообщение
    private void SendMessage(bool PID)
    {
        if (socketConnection == null)
        {
            return;
        }
        try
        {
            //Получаем стрим из сокета 			
            NetworkStream stream = socketConnection.GetStream();
            if (stream.CanWrite)
            {
                string clientMessage = GetStringMessage(PID);
                //Перевод строки в массив байт
                byte[] clientMessageAsByteArray = Encoding.ASCII.GetBytes(clientMessage);
                //Записываем в стрим массив байт               
                stream.Write(clientMessageAsByteArray, 0, clientMessageAsByteArray.Length);
                Debug.Log("Клиент отправил сообщение на сервер.");
                LogBox.text += "Клиент отправил сообщение на сервер. \n";
            }
        }
        catch (SocketException socketException)
        {
            Debug.Log("Socket exception: " + socketException);
            LogBox.text = "Исключение на сокете: " + socketException + "\n";
        }
    }
}
