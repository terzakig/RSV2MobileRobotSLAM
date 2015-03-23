using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace RobosapienRFControl
{
    class APath
    {
        // general motion instruction constants
        public const int instGO_NORTH = 0;
        public const int instGO_EAST = 1;
        public const int instGO_SOUTH = 2;
        public const int instGO_WEST = 3;
        
        // members
        public int[] amoves;
        public int pathlen;

        public APath(int plen)
        {
            pathlen = plen;
            amoves = new int[pathlen];
            
        }

        public void appendPath(APath path)
        {
            int[] tempmoves = new int[pathlen + path.pathlen];
            int i;
            
            for (i = 0; i < pathlen+path.pathlen; i++)
                tempmoves[i] = (i < pathlen) ? amoves[i] : path.amoves[i - pathlen];

            amoves = tempmoves;
            pathlen += path.pathlen;
        }
           

    }
}
