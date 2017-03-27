using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices.WindowsRuntime;
using Windows.Foundation;
using Windows.Foundation.Collections;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Controls.Primitives;
using Windows.UI.Xaml.Data;
using Windows.UI.Xaml.Input;
using Windows.UI.Xaml.Media;
using Windows.UI.Xaml.Navigation;
using MavLinkUwp;
using System.Diagnostics;
using System.Threading.Tasks;
using Windows.Devices.Enumeration;
using Windows.Devices.SerialCommunication;
using Windows.Storage.Streams;
using Windows.Storage;

// The Blank Page item template is documented at http://go.microsoft.com/fwlink/?LinkId=402352&clcid=0x409

namespace UwpDrone
{
    /// <summary>
    /// An empty page that can be used on its own or navigated to within a Frame.
    /// </summary>
    public sealed partial class MainPage : Page
    {
        private SerialDevice _device;
        private DataWriter writer = null;
        private DataReader reader = null;

        UwpMavLink mav = new UwpMavLink();
        public MainPage()
        {
            this.InitializeComponent();
        }

        protected override async void OnNavigatedTo(NavigationEventArgs e)
        {
            base.OnNavigatedTo(e);


            await connect();

            bool connected = mav.connectToMavLink(writer, reader);
            Debug.WriteLine($"Connected - {connected}");

            mav.arm();

            await Task.Delay(5000);

            mav.disarm();
        }

        public async Task connect(string identifyingSubStr = "VID_26AC")
        {
            string selector = SerialDevice.GetDeviceSelector();
            var deviceCollection = await DeviceInformation.FindAllAsync(selector);

            if (deviceCollection.Count == 0)
                return;

            for (int i = 0; i < deviceCollection.Count; ++i)
            {
                if (deviceCollection[i].Name.Contains(identifyingSubStr) || deviceCollection[i].Id.Contains(identifyingSubStr))
                {
                    _device = await SerialDevice.FromIdAsync(deviceCollection[i].Id);
                    if (_device != null)
                    {
                        _device.BaudRate = 460800;
                        _device.Parity = SerialParity.None;
                        _device.DataBits = 8;
                        _device.StopBits = SerialStopBitCount.One;
                        _device.Handshake = SerialHandshake.None;
                        _device.ReadTimeout = TimeSpan.FromSeconds(5);
                        _device.WriteTimeout = TimeSpan.FromSeconds(5);
                        //_device.IsRequestToSendEnabled = false;
                        //_device.IsDataTerminalReadyEnabled = false;


                        writer = new DataWriter(_device.OutputStream);
                        reader = new DataReader(_device.InputStream);
                        //reader.InputStreamOptions = InputStreamOptions.Partial;

                        return;
                    }
                }
            }
        }
    }
}
