using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace RobosapienRFControl
{
    class HistogramEntry
    {
        public int Input;
        public int Frequency;

        public HistogramEntry next;

        public HistogramEntry(int input)
        {
            Input = input;
            Frequency = 1;

            next = null;
        }


        public static HistogramEntry findHistogramEntry(HistogramEntry root, int input)
        {
            Boolean found = false;
            HistogramEntry temp = root;
            while ((!found) && (temp != null))
            {
                found = (temp.Input == input);
                if (!found)
                    temp = temp.next;
            }

            return temp;
        }

        public static HistogramEntry addHistogramEntry(HistogramEntry root, int input)
        {
            HistogramEntry temp = root;
            HistogramEntry newroot;
            if (temp == null)
            {
                temp = new HistogramEntry(input);
                newroot = temp;
            }
            else
            {

                newroot = root;
                while (temp.next != null)
                    temp = temp.next;

                temp.next = new HistogramEntry(input);
            }

            return newroot;
        }

        public static int getFrequency(HistogramEntry root, int input)
        {
            HistogramEntry entry = findHistogramEntry ( root, input);
            int freq;
            if (entry != null) freq = entry.Frequency;
            else freq = 0;


            return freq;
        }


        // update the Histogram or add a new entry
        public static void increaseFrequency(ref HistogramEntry root, int input)
        {
            HistogramEntry entry = findHistogramEntry (root, input);
            if (entry == null)
                root = addHistogramEntry (root, input);
            else
                entry.Frequency += 1;

        }

        public static int HistogramLength(HistogramEntry root)
        {
            int length = 0;
            HistogramEntry temp = root;

            while (temp != null)
            {
                length++;
                temp = temp.next;
            }


            return length;
        }

    }
}
