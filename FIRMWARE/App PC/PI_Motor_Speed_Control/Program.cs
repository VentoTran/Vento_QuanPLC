using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace PI_Motor_Speed_Control
{
    class Constants
    {
        public const byte ID_offset = 7;
        public const byte RW_offset = 6;
        public const byte ER_offset = 5;

        public const byte Cur_codec = 0;
        public const byte Spd_codec = 1;
        public const byte Dut_codec = 2;
        public const byte Vcc_codec = 3;
        public const byte Ena_codec = 4;

        public const byte KPR_codec = 5;
        public const byte KIR_codec = 6;
        public const byte KPC_codec = 7;
        public const byte KIC_codec = 8;
        public const byte REF_codec = 9;
        public const byte TS_codec = 10;
        public const byte mVpA_codec = 11;
        public const byte tpR_codec = 12;


        public const byte Ack_codec = 14;
        public const byte Rty_codec = 15;
    }

    static class Program
    {
        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        [STAThread]
        static void Main()
        {
            //Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            Application.Run(new Form1());
        }
    }
}
