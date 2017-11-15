using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace GameFW.Nav
{
    public interface IPathFind
    {
        List<int> GetPaths(Vector2 pos, Vector2 target);
    }
}
