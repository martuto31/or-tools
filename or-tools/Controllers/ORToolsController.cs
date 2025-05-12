using Microsoft.AspNetCore.Mvc;

using or_tools.Services;
using or_tools.Models;

namespace or_tools.Controllers
{
    [ApiController]
    [Route("[controller]")]
    public class ORToolsController : Controller
    {
        private readonly ORService _orService;

        public ORToolsController()
        {
            _orService = new ORService();
        }

        [HttpPost]
        public IActionResult Solve([FromBody] TspVrpRequest request)
        {
            var result = _orService.Solve(request);

            if (result == null)
            {
                return BadRequest("Could not solve the problem.");
            }

            return Ok(result);
        }
    }
}
