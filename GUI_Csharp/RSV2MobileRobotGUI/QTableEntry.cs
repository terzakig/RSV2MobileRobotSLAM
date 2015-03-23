using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace RobosapienRFControl
{
    class QTableEntry
    {
        public int Input, Output;
        public double QValue;

        // the process is a Non-Deterministic Markov Decision Process,
        // thus, frequency represents the number of times that
        // an input-output pair has been met inside the various IOlog entries 
        // of the metanode
        public int Frequency;

        public QTableEntry next;


        public QTableEntry(int input, int output, double qvalue)
        {
            Input = input;
            Output = output;
            QValue = qvalue;

            Frequency = 0;

            next = null;
        }


        public static QTableEntry findQTableEntry(QTableEntry root, int input, int output)
        {
            Boolean found = false;
            QTableEntry temp = root;
            while ((!found) && (temp != null)) 
            {
                found = (temp.Input == input) && (temp.Output == output);
                if (!found)
                    temp = temp.next;
            }

            return temp;
        }


        public static QTableEntry addQTableEntry(QTableEntry root, int input, int output, double qvalue)
        {
            QTableEntry temp = root;
            QTableEntry newroot;
            if (temp == null)
            {
                temp = new QTableEntry(input, output, qvalue);
                newroot = temp;
            }
            else
            {

                newroot = root;
                while (temp.next != null)
                    temp = temp.next;

                temp.next = new QTableEntry(input, output, qvalue);
            }

            return newroot;
        }


        public static double getQValue(QTableEntry root, int input, int output)
        {
            QTableEntry entry = findQTableEntry(root, input, output);
            double qvalue;
            if (entry != null) qvalue = entry.QValue;
            else qvalue = 0;


            return qvalue;
        }

        // update the Q-Table or add a new entry
        public static void assignQValue(ref QTableEntry root, int input, int output, double qvalue)
        {
            QTableEntry entry = findQTableEntry(root, input, output);
            if (entry == null)
                root = addQTableEntry(root, input, output, qvalue);
            else
                entry.QValue = qvalue;
                
        
        }

        // return the maximum Q-value for a given input. Zero if entry does not exist
        public static QTableEntry getMaxQValue(QTableEntry root, int input)
        {
            double maxvalue = 0;
            Boolean exists = false;
            QTableEntry temp = root;
            QTableEntry maxentry = null;
            while (temp != null)
            {
                if (temp.Input == input)
                    if (!exists)
                    {
                        exists = true;
                        maxvalue = temp.QValue;
                        maxentry = temp;
                    }
                    else
                    {
                        if (maxvalue < temp.QValue)
                        {
                            maxvalue = temp.QValue;
                            maxentry = temp;
                        }
                    }
                temp = temp.next;
            }

            return maxentry;
        }
    }
}
