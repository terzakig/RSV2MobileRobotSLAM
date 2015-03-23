using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace RobosapienRFControl
{
    class RewardTableEntry
    {
        public int Input;
        public double Reward;

        public RewardTableEntry next;

        public RewardTableEntry(int input, double reward)
        {
            Reward = reward;
            Input = input;

            next = null;
        }

        public static RewardTableEntry findRewardTableEntry(RewardTableEntry root, int input)
        {
            Boolean found = false;
            RewardTableEntry temp = root;
            while ((!found) && (temp != null))
            {
                found = (temp.Input == input);
                if (!found)
                    temp = temp.next;
            }

            return temp;
        }

        public static RewardTableEntry addRewardTableEntry(RewardTableEntry root, int input, double reward)
        {
            RewardTableEntry temp = root;
            RewardTableEntry newroot;
            if (temp == null)
            {
                temp = new RewardTableEntry(input, reward);
                newroot = temp;
            }
            else
            {

                newroot = root;
                while (temp.next != null)
                    temp = temp.next;

                temp.next = new RewardTableEntry(input, reward);
            }

            return newroot;
        }

        public static double getReward(RewardTableEntry root, int input)
        {
            RewardTableEntry entry = findRewardTableEntry(root, input);
            double reward;
            if (entry != null) reward = entry.Reward;
            else reward = 0;


            return reward;
        }

        // update the Reward Table or add a new entry
        public static void updateRewardEntry(ref RewardTableEntry root, int input, double reward)
        {
            RewardTableEntry entry = findRewardTableEntry(root, input);
            if (entry == null)
                root = addRewardTableEntry(root, input, reward);
            else
                entry.Reward = reward;

        }

    }
}
